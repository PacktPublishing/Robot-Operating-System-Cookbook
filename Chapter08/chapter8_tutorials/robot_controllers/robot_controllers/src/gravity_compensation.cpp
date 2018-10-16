/*********************************************************************
 * Software License Agreement (BSD License)
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

/* Author: Michael Ferguson */

#include <pluginlib/class_list_macros.h>
#include <robot_controllers/gravity_compensation.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::GravityCompensation, robot_controllers::Controller)

namespace robot_controllers
{

int GravityCompensation::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  Controller::init(nh, manager);
  manager_ = manager;

  // Load URDF
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
    return -1;
  }

  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
    return -1;
  }

  // Populate the Chain
  std::string root, tip;
  nh.param<std::string>("root", root, "torso_lift_link");
  nh.param<std::string>("tip", tip, "wrist_roll_link");
  if(!kdl_tree.getChain(root, tip, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    return -1;
  }

  kdl_chain_dynamics_.reset(new KDL::ChainDynParam(kdl_chain_, KDL::Vector(0,0,-9.81)));

  // Init positions
  positions_ = KDL::JntArrayVel(kdl_chain_.getNrOfJoints());
  KDL::SetToZero(positions_.q);
  KDL::SetToZero(positions_.qdot);

  // Init Joint Handles
  joints_.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      joints_.push_back(manager_->getJointHandle(kdl_chain_.getSegment(i).getJoint().getName()));

  initialized_ = true;

  // Should we autostart?
  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

bool GravityCompensation::start()
{
  if (!initialized_)
    return false;
  return true;
}

void GravityCompensation::update(const ros::Time& time, const ros::Duration& dt)
{
  // Need to initialize KDL structs
  if (!initialized_)
    return;

  // Get current positions
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
    positions_.q.data[i] = joints_[i]->getPosition();

  // Do the gravity compensation
  KDL::JntArray torques(kdl_chain_.getNrOfJoints());
  kdl_chain_dynamics_->JntToGravity(positions_.q, torques);

  // Update effort command
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
    joints_[i]->setEffort(torques.data[i]);
}

std::vector<std::string> GravityCompensation::getCommandedNames()
{
  std::vector<std::string> names;
  if (initialized_)
  {
    for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
      if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
        names.push_back(kdl_chain_.getSegment(i).getJoint().getName());
  }
  return names;
}

std::vector<std::string> GravityCompensation::getClaimedNames()
{
  // We don't claim anything
  std::vector<std::string> names;
  return names;
}

}  // namespace robot_controllers
