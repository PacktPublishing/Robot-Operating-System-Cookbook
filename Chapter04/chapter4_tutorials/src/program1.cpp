
#include <ros/ros.h>
#include <ros/console.h>

#define OVERRIDE_NODE_VERBOSITY_LEVEL 0

int main( int argc, char **argv )
{

  ros::init( argc, argv, "program1" );

#if OVERRIDE_NODE_VERBOSITY_LEVEL
  /* Setting the logging level manually to DEBUG */
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
#endif

  ros::NodeHandle nh;

  const double val = 3.14;

  ROS_DEBUG( "We are looking DEBUG message" );

  ROS_DEBUG( "We are looking DEBUG message with an argument: %f", val );

  ROS_DEBUG_STREAM("We are looking DEBUG stream message with an argument: " << val);

  ros::spinOnce();

  return EXIT_SUCCESS;

}

