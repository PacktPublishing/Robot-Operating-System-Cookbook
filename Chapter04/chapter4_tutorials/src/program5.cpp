
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include <chapter4_tutorials/SetSpeed.h>

float previous_speed = 0.;
float current_speed  = 0.;

void callback_temperature( const std_msgs::Int32::ConstPtr& msg )
{
    ROS_INFO_STREAM( "Temperature = " << msg->data );
}

void callback_acceleration( const geometry_msgs::Vector3::ConstPtr& msg )
{
    ROS_INFO_STREAM("Acceleration = (" << msg->x << ", " << msg->y << ", " << msg->z << ")");
}

bool callback_speed(chapter4_tutorials::SetSpeed::Request  &req, chapter4_tutorials::SetSpeed::Response &res)
{
    ROS_INFO_STREAM("Speed service request: desired speed = " << req.desired_speed);

    current_speed = 0.9 * req.desired_speed;

    res.previous_speed = previous_speed;
    res.current_speed  = current_speed;
    res.stalled        = current_speed < 0.1;

    previous_speed = current_speed;

    return true;
}


int main( int argc, char **argv )
{

    ros::init( argc, argv, "program5" );

    ros::NodeHandle nh;

    ros::Subscriber sub_temp = nh.subscribe( "temperature", 1000, callback_temperature);
    ros::Subscriber sub_accel = nh.subscribe( "acceleration", 1000, callback_acceleration);

    ros::ServiceServer srv_speed = nh.advertiseService( "speed", callback_speed );


    ros::spin();


    return EXIT_SUCCESS;

}

