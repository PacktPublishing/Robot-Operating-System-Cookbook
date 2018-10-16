/*
 * Copyright (c) 2014, Fetch Robotics Inc.
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

#ifndef ROBOT_CONTROLLERS_INTERFACE_CONTROLLER_LOADER_H
#define ROBOT_CONTROLLERS_INTERFACE_CONTROLLER_LOADER_H

#include <string>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <robot_controllers_interface/controller.h>

namespace robot_controllers
{

// Forward def
class ControllerManager;

/** @brief Class for loading and managing a single controller. */
class ControllerLoader
{
public:
  /** @brief Initialize this loader */
  ControllerLoader();

  /** @brief Load the controller. */
  bool init(const std::string& name, ControllerManager* manager);

  /** @brief This calls through to controller, saves state locally. */
  bool start();

  /** @brief This calls through to controller, saves state locally. */
  bool stop(bool force);

  /** @brief This calls through to controller. */
  bool reset();

  /** @brief If controller is active, calls through to controller. */
  void update(const ros::Time& time, const ros::Duration& dt);

  /** @brief Returns true if the controller is active. */
  bool isActive();

  /** @brief Returns the controller held by this loader. */
  ControllerPtr getController();

private:
  pluginlib::ClassLoader<robot_controllers::Controller> plugin_loader_;
  ControllerPtr controller_;
  bool active_;
};

typedef boost::shared_ptr<ControllerLoader> ControllerLoaderPtr;

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_INTERFACE_CONTROLLER_MANAGER_H
