/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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

// Author: Michael Ferguson

#ifndef ROBOT_CONTROLLERS_FOLLOW_JOINT_TRAJECTORY_H
#define ROBOT_CONTROLLERS_FOLLOW_JOINT_TRAJECTORY_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/joint_handle.h>
#include <robot_controllers_interface/controller_manager.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

#include <angles/angles.h>
#include <robot_controllers/trajectory.h>
#include <robot_controllers/trajectory_spline_sampler.h>

namespace robot_controllers
{

/**
 *  @class FollowJointTrajectoryController
 *  @brief This ROS interface implements a FollowJointTrajectoryAction
 *         interface for controlling (primarily) robot arms.
 */
class FollowJointTrajectoryController : public Controller
{
  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> server_t;

public:
  FollowJointTrajectoryController();
  virtual ~FollowJointTrajectoryController() {}

  /**
   * @brief Initialize the controller and any required data structures.
   * @param nh Node handle for this controller.
   * @param manager The controller manager instance, this is needed for the
   *        controller to get information about joints, etc.
   * @returns 0 if succesfully configured, negative values are error codes.
   */
  virtual int init(ros::NodeHandle& nh, ControllerManager* manager);

  /**
   * @brief Attempt to start the controller. This should be called only by the
   *        ControllerManager instance.
   * @returns True if successfully started, false otherwise.
   */
  virtual bool start();

  /**
   * @brief Attempt to stop the controller. This should be called only by the
   *        ControllerManager instance.
   * @param force Should we force the controller to stop? Some controllers
   *        may wish to continue running until they absolutely have to stop.
   * @returns True if successfully stopped, false otherwise.
   */
  virtual bool stop(bool force);

  /**
   * @brief Cleanly reset the controller to it's initial state. Some controllers
   *        may choose to stop themselves. This is mainly used in the case of the
   *        the robot exiting some fault condition.
   * @returns True if successfully reset, false otherwise.
   */
  virtual bool reset();

  /**
   * @brief This is the update loop for the controller.
   * @param time The system time.
   * @param dt The timestep since last call to update.
   */
  virtual void update(const ros::Time& now, const ros::Duration& dt);

  /** @brief Get the type of this controller. */
  virtual std::string getType()
  {
    return "robot_controllers/FollowJointTrajectoryController";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

private:
  /** @brief Callback for goal */
  void executeCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  /** @brief Get a trajectory point from the current position/velocity/acceleration. */
  TrajectoryPoint getPointFromCurrent(bool incl_vel, bool incl_acc, bool zero_vel);

  bool initialized_;
  ControllerManager* manager_;

  std::vector<JointHandlePtr> joints_;
  std::vector<std::string> joint_names_;
  std::vector<bool> continuous_;
  boost::shared_ptr<server_t> server_;

  boost::shared_ptr<TrajectorySampler> sampler_;
  boost::mutex sampler_mutex_;

  bool stop_with_action_;  /// should we stop this controller when the
                           /// action has terminated (or hold position)?

  bool stop_on_path_violation_;  /// should we stop this controller when
                                 /// a path tolerance has been violated?

  /*
   * In certain cases, we want to start a trajectory at our last sample,
   * for instance if we were pre-empted (as is often the case with teleop)
   * we need to use the velocity and position of the last sample as a
   * starting point.
   */
  TrajectoryPoint last_sample_;
  bool preempted_;  /// action was preempted
                    /// (has nothing to do with preempt() above).
  bool has_path_tolerance_;
  TrajectoryPoint path_tolerance_;

  bool has_goal_tolerance_;
  TrajectoryPoint goal_tolerance_;
  double goal_time_tolerance_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_FOLLOW_JOINT_TRAJECTORY_H_
