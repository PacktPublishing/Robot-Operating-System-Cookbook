/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fetch Robotics Inc.
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
 *   * Neither the name of Fetch Robotics Inc. nor the names of its
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

#include <pluginlib/class_list_macros.h>
#include <robot_controllers/scaled_mimic.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::ScaledMimicController, robot_controllers::Controller)

namespace robot_controllers
{

int ScaledMimicController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  Controller::init(nh, manager);

  // Setup Joints, Params
  std::string mimic, controlled;
  nh.param<std::string>("mimic_joint", mimic, "torso_lift_joint");
  nh.param<std::string>("controlled_joint", controlled, "bellows_joint");
  joint_to_mimic_ = manager->getJointHandle(mimic);
  joint_to_control_ = manager->getJointHandle(controlled);
  nh.param<double>("mimic_scale", scale_, 1.0);

  initialized_ = true;

  // Should we autostart?
  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

bool ScaledMimicController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("ScaledMimicController",
                    "Unable to start, not initialized.");
    return false;
  }
  return true;
}

bool ScaledMimicController::stop(bool force)
{
  // Stop me anytime
  return true;
}

bool ScaledMimicController::reset()
{
  // Nothing to do here
  return true;
}

void ScaledMimicController::update(const ros::Time& now, const ros::Duration& dt)
{
  if (!initialized_)
    return;

  joint_to_control_->setPosition(scale_*joint_to_mimic_->getPosition(), 0, 0);
}

std::vector<std::string> ScaledMimicController::getCommandedNames()
{
  std::vector<std::string> names;
  names.push_back(joint_to_control_->getName());
  return names;
}

std::vector<std::string> ScaledMimicController::getClaimedNames()
{
  // Claimed == commanded.
  return getCommandedNames();
}

}  // namespace robot_controllers
