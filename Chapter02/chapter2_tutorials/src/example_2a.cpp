#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_msg.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example2a");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<chapter2_tutorials::chapter2_msg>("chapter2_tutorials/message", 100);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    chapter2_tutorials::chapter2_msg msg;
    msg.A = 1;
    msg.B = 2;
    msg.C = 3;
    
    pub.publish(msg);
    ros::spinOnce();
   
    loop_rate.sleep();
  }
  return 0;
}
