#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_srv.h"

bool add(chapter2_tutorials::chapter2_srv::Request  &req,
         chapter2_tutorials::chapter2_srv::Response &res)
{
  res.sum = req.A + req.B;
  ROS_INFO("Request: A=%d, B=%d", (int)req.A, (int)req.B);
  ROS_INFO("Response: [%d]", (int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adder_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("chapter2_tutorials/adder", add);
  ROS_INFO("adder_server has started");
  ros::spin();

  return 0;
}
