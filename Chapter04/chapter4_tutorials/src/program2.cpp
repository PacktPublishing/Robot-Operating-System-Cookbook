
#include <ros/ros.h>
#include <ros/console.h>

int main( int argc, char **argv )
{

  ros::init( argc, argv, "program2" );

  ros::NodeHandle n;

  const double val = 3.14;

  /* Basic messages: */
  ROS_INFO( "ROS INFO message." );
  ROS_INFO( "ROS INFO message with argument: %f", val );
  ROS_INFO_STREAM( "ROS INFO stream message with argument: " << val);

  /* Named messages: */
  ROS_INFO_STREAM_NAMED("named_msg","ROS named INFO stream message; val = " << val);

  /* Conditional messages: */
  ROS_INFO_STREAM_COND(val < 0., "ROS conditional INFO stream message; val (" << val << ") < 0");
  ROS_INFO_STREAM_COND(val >= 0.,"ROS conditional INFO stream message; val (" << val << ") >= 0");

  /* Conditional Named messages: */
  ROS_INFO_STREAM_COND_NAMED(val < 0., "cond_named_msg","ROS conditional INFO stream message; val (" << val << ") < 0");
  ROS_INFO_STREAM_COND_NAMED(val >= 0., "cond_named_msg","ROS conditional INFO stream message; val (" << val << ") >= 0");

  /* Filtered messages: */
  struct ROSLowerFilter : public ros::console::FilterBase {
    ROSLowerFilter( const double& val ) : value( val ) {}

    inline virtual bool isEnabled()
    {
      return value < 0.;
    }

    double value;
  };

  struct ROSGreaterEqualFilter : public ros::console::FilterBase {
    ROSGreaterEqualFilter( const double& val ) : value( val ) {}

    inline virtual bool isEnabled()
    {
      return value >= 0.;
    }
  
    double value;
  };

  ROSLowerFilter filter_lower(val);
  ROSGreaterEqualFilter filter_greater_equal(val);

  ROS_INFO_STREAM_FILTER(
    &filter_lower,
    "ROS filter INFO stream message; val (" << val << ") < 0"
  );
  ROS_INFO_STREAM_FILTER(
    &filter_greater_equal,
    "ROS filter INFO stream message; val (" << val << ") >= 0"
  );

  /* Once messages: */
  for( int i = 0; i < 10; ++i ) {
    ROS_INFO_STREAM_ONCE(
      "ROS once INFO stream message; i = " << i
    );
  }

  /* Throttle messages: */
  for( int i = 0; i < 10; ++i ) {
    ROS_INFO_STREAM_THROTTLE(
      2,
      "ROS throttle INFO stream message; i = " << i
    );
    ros::Duration(1).sleep();
  }

  ros::spinOnce();

  return EXIT_SUCCESS;

}

