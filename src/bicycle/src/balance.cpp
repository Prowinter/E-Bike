#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/console.h>
#include <angles/angles.h>
#include "sensor_msgs/Imu.h"
#include <can_communication/CAN.h>
#include <can_communication/VESC.h>
#include <dynamic_reconfigure/server.h>
#include <bicycle/LQRConfig.h>

ros::ServiceClient CAN_Servo_Client;
ros::ServiceClient VESC_Motor_1_Client;
ros::ServiceClient VESC_Motor_2_Client;

double LQR_Kp=0.0,LQR_Kv=0.0,LQR_Ks=0.0;

void callback(bicycle::LQRConfig &config, uint32_t level) {
  LQR_Kp = config.LQR_Kp;
  LQR_Kv = config.LQR_Kv;
  LQR_Ks = config.LQR_Ks;
  // ROS_INFO("Reconfigure Request: %f %f %f", config.LQR_Kp, config.LQR_Kv, config.LQR_Ks);
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg){
  tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  can_communication::VESC srv;
  srv.request.Motor_rpm = 1000;

  //ROS_INFO("roll:%.3f ,pitch:%.3f ,yaw:%.3f",angles::to_degrees(roll),angles::to_degrees(pitch),angles::to_degrees(yaw));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "bicycle_balance");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("imu/imu/data", 1000, &imuCallback);

  ros::NodeHandle n;
  CAN_Servo_Client = n.serviceClient<can_communication::VESC>("VESC_Motor_1");
  
  dynamic_reconfigure::Server<bicycle::LQRConfig> server;
  dynamic_reconfigure::Server<bicycle::LQRConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // can_communication::VESC srv;
  // srv.request.Motor_rpm = 1000;
  // if (CAN_Servo_Client.call(srv))
  // {
  //   ROS_INFO("rpm: %d\r\n", srv.request.Motor_rpm);
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service can_server");
  //   return 1;
  // }
  ros::spin();
  return 0;
};