#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example1a");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("message", 100);
  ros::Rate loop_rate(10);
 
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello World!";
    msg.data = ss.str();
    pub.publish(msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
