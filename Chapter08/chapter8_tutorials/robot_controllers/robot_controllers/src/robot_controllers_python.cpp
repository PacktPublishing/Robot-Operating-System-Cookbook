/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Fetch Robotics Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Unbounded Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Derek King

#include <boost/python.hpp>
#include <robot_controllers/diff_drive_limiter.h>
#include <robot_controllers_msgs/DiffDriveLimiterParams.h>

namespace bp = boost::python;

using robot_controllers::DiffDriveLimiter;
using robot_controllers_msgs::DiffDriveLimiterParams;

class pyDiffDriveLimiter : public DiffDriveLimiter
{
public:
  bp::tuple limit(double desired_linear_velocity,
                  double desired_angular_velocity,
                  double last_linear_velocity,
                  double last_angular_velocity,
                  double safety_scaling,
                  double dt)
  {
    double limited_linear_velocity, limited_angular_velocity;
    DiffDriveLimiter::limit(&limited_linear_velocity,
                                &limited_angular_velocity,
                                desired_linear_velocity,
                                desired_angular_velocity,
                                last_linear_velocity,
                                last_angular_velocity,
                                safety_scaling,
                                dt);
    return bp::make_tuple(limited_linear_velocity, limited_angular_velocity);
  }


  bp::tuple calcWheelVelocities(double linear_velocity,
                                double angular_velocity)
  {
    double left_velocity, right_velocity;
    DiffDriveLimiter::calcWheelVelocities(&left_velocity,
                                              &right_velocity,
                                              linear_velocity,
                                              angular_velocity);
    return bp::make_tuple(left_velocity, right_velocity);
  }
};

BOOST_PYTHON_MODULE(robot_controllers_python)
{
  bp::class_<DiffDriveLimiterParams>("DiffDriveLimiterParams")
    .def_readwrite("max_linear_velocity", &DiffDriveLimiterParams::max_linear_velocity)
    .def_readwrite("max_linear_acceleration",
                   &DiffDriveLimiterParams::max_linear_acceleration)
    .def_readwrite("max_angular_velocity", &DiffDriveLimiterParams::max_angular_velocity)
    .def_readwrite("max_angular_acceleration",
                   &DiffDriveLimiterParams::max_angular_acceleration)
    .def_readwrite("max_wheel_velocity", &DiffDriveLimiterParams::max_wheel_velocity)
    .def_readwrite("track_width", &DiffDriveLimiterParams::track_width)
    .def_readwrite("angular_velocity_limits_linear_velocity",
                   &DiffDriveLimiterParams::angular_velocity_limits_linear_velocity)
    .def_readwrite("scale_to_wheel_velocity_limits",
                   &DiffDriveLimiterParams::scale_to_wheel_velocity_limits)
    ;

  bp::class_<pyDiffDriveLimiter>("DiffDriveLimiter")
    .def("limit", &pyDiffDriveLimiter::limit)
    .def("calcWheelVelocities", &pyDiffDriveLimiter::calcWheelVelocities)
    .add_property("params",
                  &pyDiffDriveLimiter::getParams,
                  &pyDiffDriveLimiter::setParams)
    .def("setParams", &pyDiffDriveLimiter::setParams)
    .def("getParams", &pyDiffDriveLimiter::getParams)
    .def("getDefaultParams", &pyDiffDriveLimiter::getDefaultParams)
    .staticmethod("getDefaultParams")
    .def("validateParams", &pyDiffDriveLimiter::validateParams)
    .staticmethod("validateParams")
    ;
}
