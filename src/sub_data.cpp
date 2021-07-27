#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "dsk.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void gyro_callback(const sensor_msgs::Imu::ConstPtr& msg){
  ROS_INFO("gyro_callback: [%f]", msg->angular_velocity.x);
}
void mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg){
  ROS_INFO("123");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_data");
  ros::NodeHandle n;
  ros::Publisher data_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub1 = n.subscribe("/mavros/imu/data_raw", 1, gyro_callback);
  ros::Subscriber sub2 = n.subscribe("/mavros/imu/mag", 1, mag_callback);
  while(ros::ok()){
    ros::spinOnce();
    Eigen::Vector4d q;
    float normq=0.0;
    normq=sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
    q(1)=q(1)/normq;
    q(2)=q(2)/normq;
    q(3)=q(3)/normq;
    q(4)=q(4)/normq;






  }


  return 0;
}