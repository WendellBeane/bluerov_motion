#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"



class twist_controller
{
  float linx_vel, goal_xvel; 
public:
  void twist_callback(const geometry_msgs::Twist& msg);

public:
  void imu_callback(const sensor_msgs::Imu& msg);

};


void twistCallback(const geometry_msgs::Twist& msg){
  float linx, linz, angZ;
  linx = msg.linear.x;
  angZ = msg.angular.z;
  linz = msg.linear.z;

  ROS_INFO("linearx is : [%f]", linx)
  ROS_INFO("linearz is : [%f]", linz)
  ROS_INFO("angularz is : [%f]", angZ)

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_listener");

  ros::NodeHandle n;

  ros::Subscriber twist_sub = n.subscribe("bluerov/cmd_vel", 1000, twistCallback)

  ros::Subscriber imu_sub = n.subscribe("/mavros/imu/data", 1000, imuCallback)

  ros:spin()

  return 0;

}
