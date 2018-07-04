
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <chapter4_tutorials/DynamicParamConfig.h>

class DynamicParamServer
{
public:
    DynamicParamServer()
    {
        _cfg_server.setCallback(boost::bind(&DynamicParamServer::callback, this, _1, _2));
    }

    void callback(chapter4_tutorials::DynamicParamConfig& config, uint32_t level)
    {
        ROS_INFO_STREAM(
                    "New configuration received with level = " << level << ":\n" <<
                    "BOOL   = " << config.BOOL << "\n" <<
                    "INT    = " << config.INT<< "\n" <<
                    "DOUBLE = " << config.DOUBLE << "\n" <<
                    "STRING = " << config.STRING
                    );
    }

private:
    dynamic_reconfigure::Server<chapter4_tutorials::DynamicParamConfig> _cfg_server;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "program6");

    DynamicParamServer dps;

    while(ros::ok())
    {
        ros::spin();
    }

    return EXIT_SUCCESS;
}
