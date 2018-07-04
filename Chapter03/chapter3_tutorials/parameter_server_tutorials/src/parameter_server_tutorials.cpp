#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <parameter_server_tutorials/parameter_server_Config.h>

void callback(parameter_server_tutorials::parameter_server_Config &config, uint32_t level) {

  ROS_INFO("Reconfigure Request: %s %d %f %s %d", 
            config.BOOL_PARAM?"True":"False", 
            config.INT_PARAM, 
            config.DOUBLE_PARAM, 
            config.STR_PARAM.c_str(),
            config.SIZE);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "parameter_server_tutorials");

  dynamic_reconfigure::Server<parameter_server_tutorials::parameter_server_Config> server;
  dynamic_reconfigure::Server<parameter_server_tutorials::parameter_server_Config>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning");
  ros::spin();
  return 0;
}
