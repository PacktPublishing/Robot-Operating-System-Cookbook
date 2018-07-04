#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_msg.h"

void messageCallback(const chapter2_tutorials::chapter2_msg::ConstPtr& msg)
{
  ROS_INFO("I have received: [%d] [%d] [%d]", msg->A, msg->B, msg->C);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example3_b");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chapter2_tutorials/message", 100, messageCallback);
  ros::spin();
  return 0;
}
