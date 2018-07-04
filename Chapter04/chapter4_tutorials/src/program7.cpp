
#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>

class DiagnosticNode
{
public:
    DiagnosticNode() : _iterator(0)
    {
        _diagnostic.add("status", this, &DiagnosticNode::diagnostics);
        _diagnostic.setHardwareID("RosCookBook");
    }

    void update()
    {
        _diagnostic.update();
    }

    void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
    {
        if (_iterator % 2 == 0)
            status.summary(0, "OK");
        else
            status.summary(2, "Error");

        status.add("Index", _iterator);

        ROS_DEBUG_THROTTLE(1, "Diagnostics published");

        ++_iterator;
    }

private:
    diagnostic_updater::Updater _diagnostic;

    int _iterator;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "program7");

    DiagnosticNode dn;

    ros::Rate rate(10.0);
    while(ros::ok())
    {
        dn.update();

        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
