
#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#define Offset_Pitch  89.853f
#define Offset_Yaw  97.853f
#define Offset_Roll 0.985f
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 将 IMU 的欧拉角转换为四元数
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(pitch, roll, yaw);  //change direction
    tf::Quaternion q1;
    q1.setRPY(roll-angles::from_degrees(Offset_Roll), pitch+angles::from_degrees(Offset_Pitch)+0.17453, yaw+angles::from_degrees(Offset_Yaw));
    
    // 创建 TF 变换，将 IMU 的姿态绑定到机器人节点上
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.068861,-0.00088133,0.018549));
    transform.setRotation(q1);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "j2", "j6"));




    ROS_INFO("roll:%.3f ,pitch:%.3f ,yaw:%.3f",angles::to_degrees(roll)-Offset_Roll,angles::to_degrees(pitch)+Offset_Pitch,angles::to_degrees(yaw));
    // sensor_msgs::JointState joint_state;
    // static tf::TransformListener listener;
    // tf::StampedTransform stamped_transform;
    // try
    // {
    //     listener.lookupTransform("j6", "j2", ros::Time(0), stamped_transform);
    //     stamped_transform.getRotation();
    // }
    // catch (tf::TransformException& ex)
    // {
    //     ROS_ERROR_STREAM("Failed to lookup transform: " << ex.what());
    //     return;
    // }

    // // 发布修改后的关节状态
    // ros::NodeHandle nh;
    // ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    // joint_state_pub.publish(joint_state);

}

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "imu_node");

    // 创建 ROS 句柄
    ros::NodeHandle nh;

    // 创建 IMU 订阅器
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/imu/data", 10, imuCallback);

    // 循环等待消息
    ros::spin();

    return 0;
}





