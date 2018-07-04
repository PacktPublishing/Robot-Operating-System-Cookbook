#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_srv.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adder_client");
  if (argc != 3)
  {
    ROS_INFO("Usage: adder_client A B ");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<chapter2_tutorials::chapter2_srv>("chapter2_tutorials/adder");
  chapter2_tutorials::chapter2_srv srv;
  srv.request.A = atoll(argv[1]);
  srv.request.B = atoll(argv[2]);
  
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service adder_server");
    return 1;
  }

  return 0;
}
