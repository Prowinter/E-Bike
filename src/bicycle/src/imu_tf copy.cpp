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
#define _DEBUG_IMU 1

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
  ROS_INFO("roll:%.3f ,pitch:%.3f ,yaw:%.3f",angles::to_degrees(roll),angles::to_degrees(pitch),angles::to_degrees(yaw));
  #endif
}

int main(int argc, char** argv){
  ros::init(argc, argv, "bicycle_balance");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("imu/imu/data", 1000, &imuCallback);

  ros::spin();
  return 0;
};