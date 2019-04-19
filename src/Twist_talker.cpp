#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_talker");

  ros::NodeHandle n;

  ros::Publisher tiwt_pub = n.advertise<geometry_msgs::Twist>("bluerov/cmd_vel", 1000);

  ros::Rate rate(2);
}
