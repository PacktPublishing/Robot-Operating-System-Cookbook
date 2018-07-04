
#include <ros/ros.h>
#include <ros/console.h>

int main( int argc, char **argv )
{

  ros::init( argc, argv, "program1_dump" );

  ros::NodeHandle nh;

  const double val = 3.14;
  unsigned int *coreDump = NULL;

  ROS_DEBUG( "We would look Core Dump Demo" ); 

  *coreDump = 0x100;

  ROS_DEBUG( "We are lookig DEBUG message with an argument: %f", val );

  ROS_DEBUG_STREAM("We are looking DEBUG stream message with an argument: " << val);

  ros::spinOnce();

  return EXIT_SUCCESS;

}

