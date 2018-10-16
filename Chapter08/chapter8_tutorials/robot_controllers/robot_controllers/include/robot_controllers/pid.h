/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef ROBOT_CONTROLLERS_PID_H
#define ROBOT_CONTROLLERS_PID_H

#include <ros/ros.h>

namespace robot_controllers
{

class PID
{
public:
  /**
   *  @brief Constructor
   *  @param p proportional gain
   *  @param i integral gain
   *  @param d derivative gain
   *  @param i_max integral wind-up max value
   *  @param i_min integral wind-up min value
   */
  PID(double p, double i, double d, double i_max, double i_min);

  /** @brief Constructor. Starts all gains and limits at zero */
  PID();

  /** @brief initialize gain settings from ROS parameter values */
  bool init(const ros::NodeHandle &nh);

  /** @brief Reset integral wind-up term */
  void reset();

  /**
   *  @brief Run PID calculation and return control result
   *  @param error error term (goal-actual)
   *  @param dt timestep used for calculating derivative term and integral windup
   *  @returns calculated PID command
   * 
   *  PID derivate term is calculated from change in error value and dt   
   */
  double update(double error, double dt);

  /**
   *  @brief Run PID calculation and return control result
   *  @param error error term (goal-actual)
   *  @param error_dot value that is used for derivative term calculation
   *  @param dt timestep used for integral windup
   *  @returns calculated PID command
   */  
  double update(double error, double error_dot, double dt);

protected:

  /**
   *  @brief Checks and fixes gain settings
   *  @returns true is gains are ok, false if gains are invalid
   */
  bool checkGains();

  /// proportial gain
  double p_gain_;
  /// integral gain
  double i_gain_;
  /// derivative gain
  double d_gain_;
  /// integral gain min and max limits
  double i_max_, i_min_;

  /// integral wind-up term
  double i_term_; 

  /// Last error value, used for calculating error_dot when not provided
  double error_last_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_PID_H
