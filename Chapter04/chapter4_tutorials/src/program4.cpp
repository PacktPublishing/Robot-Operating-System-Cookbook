
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include <chapter4_tutorials/SetSpeed.h>

int main( int argc, char **argv )
{

    ros::init( argc, argv, "program4" );

    ros::NodeHandle nh;

    ros::Publisher pub_temp = nh.advertise< std_msgs::Int32 >( "temperature", 1000 );
    ros::Publisher pub_accel = nh.advertise< geometry_msgs::Vector3 >( "acceleration", 1000 );

    ros::ServiceClient srv_speed = nh.serviceClient< chapter4_tutorials::SetSpeed>( "speed" );

    std_msgs::Int32 msg_temp;
    geometry_msgs::Vector3 msg_accel;
    chapter4_tutorials::SetSpeed msg_speed;

    int i = 0;

    ros::Rate rate( 1 );
    while( ros::ok() ) {

        msg_temp.data = i;

        msg_accel.x = 0.1 * i;
        msg_accel.y = 0.2 * i;
        msg_accel.z = 0.3 * i;

        msg_speed.request.desired_speed = 0.01 * i;

        pub_temp.publish( msg_temp );
        pub_accel.publish( msg_accel );

        if( srv_speed.call( msg_speed ) )
        {
            ROS_INFO_STREAM(
                        "SetSpeed response:\n" <<
                        "Previous speed = " << msg_speed.response.previous_speed << "\n" <<
                        "Current  speed = " << msg_speed.response.current_speed      << "\n" <<
                        "Motor stalled  = " << (msg_speed.response.stalled ? "true" : "false" )
                        );
        }
        else
        {
            /* Note that this might happen at the beginning, because
               the service server could have not started yet! */
            ROS_ERROR_STREAM( "Call to speed service failed!" );
        }

        ++i;

        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;

}

