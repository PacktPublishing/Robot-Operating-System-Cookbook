/*
 * Copyright (c) 2014-2016, Fetch Robotics Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fetch Robotics Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Michael Ferguson

#ifndef ROBOT_CONTROLLERS_INTERFACE_CONTROLLER_MANAGER_H
#define ROBOT_CONTROLLERS_INTERFACE_CONTROLLER_MANAGER_H

#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <robot_controllers_msgs/QueryControllerStatesAction.h>

#include <robot_controllers_interface/joint_handle.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/controller_loader.h>

namespace robot_controllers
{

/** @brief Base class for a controller manager. */
class ControllerManager
{
  typedef actionlib::SimpleActionServer<robot_controllers_msgs::QueryControllerStatesAction> server_t;

  typedef std::vector<ControllerLoaderPtr> ControllerList;
  typedef std::vector<JointHandlePtr> JointHandleList;

public:
  ControllerManager();

  /** @brief Ensure proper shutdown with virtual destructor. */
  virtual ~ControllerManager()
  {
  }

  /**
   * @brief Startup the controller manager, loading default controllers.
   * @param nh The proper node handle for finding parameters.
   * @returns 0 if success, negative values are error codes.
   *
   * Note: JointHandles should be added before this is called.
   */
  virtual int init(ros::NodeHandle& nh);

  /** @brief Start a controller. */
  virtual int requestStart(const std::string& name);

  /** @brief Stop a controller. */
  virtual int requestStop(const std::string& name);

  /** @brief Update active controllers. */
  virtual void update(const ros::Time& time, const ros::Duration& dt);

  /** @brief Reset all controllers. */
  virtual void reset();

  /** @brief Add a joint handle. */
  bool addJointHandle(JointHandlePtr& j);

  /**
   * @brief Get the handle associated with a particular joint/controller name.
   * @param name The name of the joint/controller.
   */
  HandlePtr getHandle(const std::string& name);

  /**
   * @brief Get the joint handle associated with a particular joint name.
   * @param name The name of the joint.
   *
   * This is mainly a convienence function.
   */
  JointHandlePtr getJointHandle(const std::string& name);

private:
  /** @brief Action callback. */
  void execute(const robot_controllers_msgs::QueryControllerStatesGoalConstPtr& goal);

  /** @brief Fill in the current state of controllers. */
  void getState(robot_controllers_msgs::QueryControllerStatesResult& result);

  /** @brief Load a controller. */
  bool load(const std::string& name);

  ControllerList controllers_;
  JointHandleList joints_;

  boost::shared_ptr<server_t> server_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_INTERFACE_CONTROLLER_MANAGER_H
