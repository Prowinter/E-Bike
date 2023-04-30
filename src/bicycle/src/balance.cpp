#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/console.h>
#include <angles/angles.h>
#include "sensor_msgs/Imu.h"
#include <can_communication/CAN_srv.h>
#include <can_communication/VESC_srv.h>
#include <can_communication/Odrive_srv.h>
#include <can_communication/Odrive_msg.h>
#include <can_communication/Bicycle_msg.h>
#include <dynamic_reconfigure/server.h>
#include <bicycle/LQRConfig.h>
#include <std_msgs/Float32.h>
#include <ros_odrive/odrive_ctrl.h>
#include <bicycle/Bicycle_Roll.h>

#define Time_Sec 0.006f

// #define _DEBUG_IMU 1
// #define _DEBUG_Dynamic_LQR 1

// #define USE_USB_Motor_Control 1

#define CAN_ID 0x0E
#define VESC_ID_1 0x68
#define VESC_ID_2 0x69

#define Angle_Offset 1.5f

enum commands {
    CMD_AXIS_RESET,
    CMD_AXIS_IDLE,
    CMD_AXIS_CLOSED_LOOP,
    CMD_AXIS_SET_VELOCITY,
    CMD_AXIS_SET_VELOCITY_DUAL,
    CMD_REBOOT
};

struct BikeStructure{             // Structure declaration
  double LQR_Kp=1806.1;
  double LQR_Kv=1903.5;
  double LQR_Ks=0.0102;
  int Servo_Direction_Angle;
  float Drive_Motor_duty;
  float Balance_Motor_rpm;
  float Bicycle_voltage;
  double Roll_Angular_Velocity = 0.0f;
  double Roll;
  double Roll_Gyro;
  bool Switch;
} Ebike;     // Structure variable


ros::ServiceClient CAN_Servo_Client;
ros::ServiceClient VESC_Motor_1_Client;
ros::ServiceClient VESC_Motor_2_Client;
ros::ServiceClient Odrive_Motor_Client;
ros::Publisher IMU_ROLL_Pbu;
ros::Publisher Odrive_pub;
ros::Publisher Roll_pub;
ros::Publisher Roll_Velocity_pub;
ros::Timer timer;

void callback(bicycle::LQRConfig &config, uint32_t level) {
  Ebike.LQR_Kp = config.LQR_Kp;
  Ebike.LQR_Kv = config.LQR_Kv;
  Ebike.LQR_Ks = config.LQR_Ks;
  Ebike.Switch = config.LQR_Switch;
}

void timerCallback(const ros::TimerEvent& event)
{
}

void CAN_Callback(const can_communication::Bicycle_msg& msg){
  if(msg.Device_ID == CAN_ID){
    Ebike.Servo_Direction_Angle = msg.Servo_Direction;
  }
}

void Motor_1_Callback(const can_communication::Bicycle_msg& msg){
  static int flag = 0;
  double Temp_Roll_Gyro ;
  if(msg.Device_ID == VESC_ID_1){
    Ebike.Balance_Motor_rpm = msg.Motor_rpm;
    Ebike.Bicycle_voltage = msg.Bicycle_voltage;
  }
}

void Motor_2_Callback(const can_communication::Bicycle_msg& msg){
  if(msg.Device_ID == VESC_ID_2){
    Ebike.Drive_Motor_duty = msg.Motor_duty;
  }
}

void Odrive_Motor_Callback(const can_communication::Odrive_msg& msg){
  Ebike.Balance_Motor_rpm = msg.Motor_rpm;
  Ebike.Bicycle_voltage   = msg.Motor_voltage;
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg){
  
  tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);  //change direction
  Ebike.Roll = - angles::to_degrees(roll)+Angle_Offset;
  Ebike.Roll_Angular_Velocity = - msg->angular_velocity.x;

  bicycle::Bicycle_Roll roll_msg;
  roll_msg.Roll = Ebike.Roll;
  roll_msg.Roll_Velocitys = Ebike.Roll_Angular_Velocity;
  Roll_pub.publish(roll_msg);

  #ifdef _DEBUG_IMU
  ROS_WARN("roll:%.3f ,pitch:%.3f ,yaw:%.3f",angles::to_degrees(roll),angles::to_degrees(pitch)-Angle_Offset,angles::to_degrees(yaw));
  #endif

  int Motor_Set_Rpm =(int)(Ebike.Roll * Ebike.LQR_Kp + Ebike.Roll_Angular_Velocity * Ebike.LQR_Kv + Ebike.Balance_Motor_rpm * Ebike.LQR_Ks);
  Motor_Set_Rpm = (Motor_Set_Rpm < 20) ? (Motor_Set_Rpm > -20) ? Motor_Set_Rpm : -20 : 20;

  #ifdef USE_USB_Motor_Control
  if(Ebike.Switch){
    ros_odrive::odrive_ctrl msgg;
    msgg.command = CMD_AXIS_SET_VELOCITY;
    msgg.axis = 0;
    msgg.fval = - Motor_Set_Rpm;
    Odrive_pub.publish(msgg);
  }else
  {
    ros_odrive::odrive_ctrl msgg;
    msgg.command = CMD_AXIS_SET_VELOCITY;
    msgg.axis = 0;
    msgg.fval = 0;
    Odrive_pub.publish(msgg);

  }
  #else
  if(Ebike.Switch){
    can_communication::Odrive_srv srv;
    srv.request.Motor_vel = Motor_Set_Rpm;
    if(!Odrive_Motor_Client.call(srv)) ROS_ERROR("Failed to call service Odrive_Motor_Client");
  }else
  {
    can_communication::Odrive_srv srv;
    srv.request.Motor_vel = 0;
    if(!Odrive_Motor_Client.call(srv)) ROS_ERROR("Failed to call service Odrive_Motor_Client");
  }
  #endif
}

int main(int argc, char** argv){
  ros::init(argc, argv, "bicycle_balance");

  // ros::NodeHandle nn;
  // IMU_ROLL_Pbu = nn.advertise<std_msgs::Float32>("/Bicycle/balance/imu_roll", 30);
  ros::NodeHandle roll_node;
  Roll_pub = roll_node.advertise<bicycle::Bicycle_Roll>("/bicycle/fdi/imu", 100);


  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("fdi_imu", 100, &imuCallback);

  // subscribe msg
  ros::NodeHandle can_node;
  ros::Subscriber can_sub = can_node.subscribe("Bicycle/CAN_Servo_info", 1000, &CAN_Callback);
  ros::NodeHandle motor_1_node;
  ros::Subscriber motor_1sub = motor_1_node.subscribe("Bicycle/VESC_Motor_1_info", 1000, &Motor_1_Callback);
  ros::NodeHandle motor_2_node;
  ros::Subscriber motor_2sub = motor_2_node.subscribe("Bicycle/VESC_Motor_2_info", 1000, &Motor_2_Callback);
  ros::NodeHandle odrive_node;
  ros::Subscriber odrive_sub = odrive_node.subscribe("Bicycle/Odrive_Motor_info", 1000, &Odrive_Motor_Callback);

  // subscribe srv
  ros::NodeHandle service_a;
  CAN_Servo_Client = service_a.serviceClient<can_communication::CAN_srv>("CAN_Servo");
  ros::NodeHandle service_b;
  VESC_Motor_1_Client = service_b.serviceClient<can_communication::VESC_srv>("VESC_Motor_1");
  // ros::NodeHandle service_c;
  // VESC_Motor_2_Client = service_c.serviceClient<can_communication::VESC_srv>("VESC_Motor_2");
  #ifdef USE_USB_Motor_Control
  ros::NodeHandle odrive_node;
  Odrive_pub = odrive_node.advertise<ros_odrive::odrive_ctrl>("/ros_odrive/odrive_ctrl_355E30753533", 100);
  #else
  ros::NodeHandle service_d;
  Odrive_Motor_Client = service_d.serviceClient<can_communication::Odrive_srv>("Odrive_Motor");
  while(!ros::service::exists("Odrive_Motor", false));
  #endif


  dynamic_reconfigure::Server<bicycle::LQRConfig> server;
  dynamic_reconfigure::Server<bicycle::LQRConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);


  // ros::NodeHandle nh;
  // timer = nh.createTimer(ros::Duration(Time_Sec), timerCallback);
  // timer.stop();

  ros::spin();
  return 0;
};