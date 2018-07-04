#include "ros/ros.h"
#include "std_msgs/String.h"

void messageCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Thanks: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example1b");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("message", 100, messageCallback);
  ros::spin();
  return 0;
}
