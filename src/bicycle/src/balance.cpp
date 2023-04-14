#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/console.h>
#include <angles/angles.h>
#include "sensor_msgs/Imu.h"
#include <can_communication/CAN.h>
#include <can_communication/VESC.h>
#include <can_communication/Bicycle.h>
#include <dynamic_reconfigure/server.h>
#include <bicycle/LQRConfig.h>
#include <std_msgs/Float32.h>

#define Angle_Offset 0.3f
#define Time_Sec 0.05f
// #define _DEBUG_IMU 1
#define _DEBUG_Dynamic_LQR 1
// #define _DEBUG_VESC_Motor_Speed 1

#define CAN_ID 0x0E
#define VESC_ID_1 0x68
#define VESC_ID_2 0x69

struct BikeStructure{             // Structure declaration
  double LQR_Kp=1806.1;
  double LQR_Kv=1903.5;
  double LQR_Ks=0.0102;
  int Servo_Direction_Angle;
  float Drive_Motor_duty;
  int   Balance_Motor_rpm;
  float Bicycle_voltage;
  double Roll_Angular_Velocity = 0.0f;
  double Roll;
} Ebike;     // Structure variable

ros::ServiceClient CAN_Servo_Client;
ros::ServiceClient VESC_Motor_1_Client;
ros::ServiceClient VESC_Motor_2_Client;
ros::Publisher IMU_ROLL_Pbu;
ros::Timer timer;

bool Switch = false;

void callback(bicycle::LQRConfig &config, uint32_t level) {
  Ebike.LQR_Kp = config.LQR_Kp;
  Ebike.LQR_Kv = config.LQR_Kv;
  Ebike.LQR_Ks = config.LQR_Ks;
  Switch = config.LQR_Switch;
  ROS_WARN("Reconfigure Request: %f %f %f %d", Ebike.LQR_Kp, Ebike.LQR_Kv, Ebike.LQR_Ks, Switch);
  // #ifdef _DEBUG_IMU
  // ROS_INFO("Reconfigure Request: %f %f %f", Ebike.LQR_Kp, Ebike.LQR_Kv, Ebike.LQR_Ks);
  // #endif
}

void timerCallback(const ros::TimerEvent& event)
{
  static double last_ang;
  Ebike.Roll_Angular_Velocity = (Ebike.Roll-last_ang)/Time_Sec;
  ROS_WARN("Roll:%f", Ebike.Roll);
  last_ang = Ebike.Roll;
  if(Switch){
    can_communication::VESC srv;
    srv.request.Motor_rpm = 1000;
    if(!VESC_Motor_1_Client.call(srv))
    {
      ROS_ERROR("Failed to call service VESC_Motor_1_Client");
    }
  }
}

void CAN_Callback(const can_communication::Bicycle& msg){
  if(msg.Device_ID == CAN_ID){
    Ebike.Servo_Direction_Angle = msg.Servo_Direction;
  }
}

void Motor_1_Callback(const can_communication::Bicycle& msg){
  static int flag = 0;
  if(msg.Device_ID == VESC_ID_1){
    Ebike.Balance_Motor_rpm = msg.Motor_rpm;
    Ebike.Bicycle_voltage = msg.Bicycle_voltage;
    Ebike.Roll = msg.Motor_Roll;
    ROS_INFO("Roll:%f", Ebike.Roll);
    if(!flag) {flag = 1;timer.start();}
  }
}

void Motor_2_Callback(const can_communication::Bicycle& msg){
  
  if(msg.Device_ID == VESC_ID_2){
    Ebike.Drive_Motor_duty = msg.Motor_duty;
  }
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg){

  
  tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(pitch, roll, yaw);  //change direction

  #ifdef _DEBUG_IMU
  // ROS_INFO("roll:%.3f ,pitch:%.3f ,yaw:%.3f",angles::to_degrees(roll)-Angle_Offset,angles::to_degrees(pitch),angles::to_degrees(yaw));
  // ROS_INFO("Balance_Motor_rpm:%d, Drive_Motor_duty:%.3f, Bicycle_voltage:%.3f, Servo_Direction_Angle:%d",  \
  //           Ebike.Balance_Motor_rpm, \
  //           Ebike.Drive_Motor_duty,  \
  //           Ebike.Bicycle_voltage,   \
  //           Ebike.Servo_Direction_Angle);
  #endif

  std_msgs::Float32 roll_msg;
  roll_msg.data = angles::to_degrees(roll);
  Ebike.Roll = angles::to_degrees(roll);
  IMU_ROLL_Pbu.publish(roll_msg);
  
  int Motor_Set_Rpm =(int)((angles::to_degrees(roll) - Angle_Offset) * Ebike.LQR_Kp + Ebike.Roll_Angular_Velocity * Ebike.LQR_Kv + Ebike.Balance_Motor_rpm * Ebike.LQR_Ks);
  // ROS_INFO("p:%.3f ,v:%.3f\r\n",Ebike.LQR_Kp,Ebike.LQR_Kv);
  
  // #ifdef _DEBUG_IMU
  // ROS_INFO("Motor_Speed:%.3f",VESC_Motor_1_Client.);
  // #endif
  ROS_INFO("rpm: %d\r\n", Motor_Set_Rpm);

  can_communication::VESC srv;
  srv.request.Motor_rpm = Motor_Set_Rpm;
  if(!VESC_Motor_1_Client.call(srv))
  {
    ROS_ERROR("Failed to call service VESC_Motor_1_Client");
  }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "bicycle_balance");

  // ros::NodeHandle nn;
  // IMU_ROLL_Pbu = nn.advertise<std_msgs::Float32>("/Bicycle/balance/imu_roll", 30);

  // ros::NodeHandle node;
  // ros::Subscriber sub = node.subscribe("imu/imu/data", 1000, &imuCallback);

  ros::NodeHandle can_node;
  ros::Subscriber can_sub = can_node.subscribe("Bicycle/CAN_Servo_info", 1000, &CAN_Callback);
  ros::NodeHandle motor_1_node;
  ros::Subscriber motor_1sub = motor_1_node.subscribe("Bicycle/VESC_Motor_1_info", 1000, &Motor_1_Callback);
  ros::NodeHandle motor_2_node;
  ros::Subscriber motor_2sub = motor_2_node.subscribe("Bicycle/VESC_Motor_2_info", 1000, &Motor_2_Callback);

  ros::NodeHandle service_a;
  CAN_Servo_Client = service_a.serviceClient<can_communication::CAN>("CAN_Servo");
  ros::NodeHandle service_b;
  VESC_Motor_1_Client = service_b.serviceClient<can_communication::VESC>("VESC_Motor_1");
  ros::NodeHandle service_c;
  VESC_Motor_2_Client = service_c.serviceClient<can_communication::VESC>("VESC_Motor_2");
  
  dynamic_reconfigure::Server<bicycle::LQRConfig> server;
  dynamic_reconfigure::Server<bicycle::LQRConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::NodeHandle nh;
  timer = nh.createTimer(ros::Duration(Time_Sec), timerCallback);
  timer.stop();

  ros::spin();
  return 0;
};