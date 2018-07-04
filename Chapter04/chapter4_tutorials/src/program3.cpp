
#include <ros/ros.h>
#include <ros/console.h>

int main( int argc, char **argv )
{

    ros::init( argc, argv, "program3" );

    ros::NodeHandle nh;

    ros::Rate rate(1);

    while(ros::ok())
    {

        ROS_DEBUG_STREAM( "ROS DEBUG message.");
        ROS_INFO_STREAM ( "ROS INFO message.");
        ROS_WARN_STREAM ( "ROS WARN message." );
        ROS_ERROR_STREAM( "ROS ERROR message." );
        ROS_FATAL_STREAM( "ROS FATAL message." );

        ROS_INFO_STREAM_NAMED( "named_msg", "ROS INFO named message." );

        ROS_INFO_STREAM_THROTTLE(2, "ROS INFO Throttle message." );

        ros::spinOnce();
        rate.sleep();
    }
    return EXIT_SUCCESS;
}

