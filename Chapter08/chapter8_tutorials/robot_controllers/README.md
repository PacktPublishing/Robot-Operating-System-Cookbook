# Robot Controllers

This is a robot control infrastructure, developed initially for Fetch and Freight, but
designed to be robot-agnostic. In comparison to ros_control, robot_controllers offers
the ability to "stack" controllers and avoids template-based hardware interfaces.
robot_controllers has not be designed with real-time constraints in mind, and is intended
primarily for robots where the real-time joint-level controllers are run in hardware,
as is the case for Fetch and Freight.

 * Devel Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Idev__robot_controllers__ubuntu_trusty_amd64)](http://build.ros.org/view/Idev/job/Idev__robot_controllers__ubuntu_trusty_amd64/)
 * AMD64 Debian Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__robot_controllers_interface__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Ibin_uT64/job/Ibin_uT64__robot_controllers_interface__ubuntu_trusty_amd64__binary/)
