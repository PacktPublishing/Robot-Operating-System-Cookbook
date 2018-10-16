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

/*
 * Derived a bit from pr2_controllers/cartesian_pose_controller.cpp
 * Author: Michael Ferguson, Wim Meeussen
 */

#include <pluginlib/class_list_macros.h>
#include <robot_controllers/cartesian_pose.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <tf_conversions/tf_kdl.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::CartesianPoseController, robot_controllers::Controller)

namespace robot_controllers
{

CartesianPoseController::CartesianPoseController() :
    initialized_(false)
{
}

int CartesianPoseController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  Controller::init(nh, manager);
  manager_ = manager;

  // Initialize KDL structures
  std::string tip_link;
  nh.param<std::string>("root_name", root_link_, "torso_lift_link");
  nh.param<std::string>("tip_name", tip_link, "gripper_link");

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

  // Populate the chain
  if(!kdl_tree.getChain(root_link_, tip_link, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    return -1;
  }

  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_delta_.resize(kdl_chain_.getNrOfJoints());
  jacobian_.resize(kdl_chain_.getNrOfJoints());

  // Initialize controllers
  robot_controllers::PID pid_controller;
  if (!pid_controller.init(ros::NodeHandle(nh,"fb_trans"))) return false;
  for (unsigned int i = 0; i < 3; i++)
    pid_.push_back(pid_controller);
  if (!pid_controller.init(ros::NodeHandle(nh,"fb_rot"))) return false;
  for (unsigned int i = 0; i < 3; i++)
    pid_.push_back(pid_controller);

  // Init joint handles
  joints_.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      joints_.push_back(manager_->getJointHandle(kdl_chain_.getSegment(i).getJoint().getName()));

  // Subscribe to command
  command_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("command", 1,
                    boost::bind(&CartesianPoseController::command, this, _1));
  last_command_ = ros::Time(0);

  // Feedback of twist
  feedback_pub_ = nh.advertise<geometry_msgs::Twist>("feedback", 10);

  initialized_ = true;
  return 0;
}

bool CartesianPoseController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("CartesianPoseController",
                    "Unable to start, not initialized.");
    return false;
  }

  if (ros::Time::now() - last_command_ > ros::Duration(3.0))
  {
    ROS_ERROR_NAMED("CartesianPoseController",
                    "Unable to start, no goal.");
    return false;
  }

  return true;
}

bool CartesianPoseController::stop(bool force)
{
  // Always stop
  return true;
}

bool CartesianPoseController::reset()
{
  // Simply stop
  return (manager_->requestStop(getName()) == 0);
}

void CartesianPoseController::update(const ros::Time& now, const ros::Duration& dt)
{
  // Need to initialize KDL structs
  if (!initialized_)
    return;  // Should never actually hit this

  // Get current pose
  actual_pose_ = getPose();

  // Pose feedback
  twist_error_ = KDL::diff(actual_pose_, desired_pose_);
  geometry_msgs::Twist t;
  t.linear.x = twist_error_(0);
  t.linear.y = twist_error_(1);
  t.linear.z = twist_error_(2);
  t.angular.x = twist_error_(3);
  t.angular.y = twist_error_(4);
  t.angular.z = twist_error_(5);
  feedback_pub_.publish(t);

  // Update PID
  for (size_t i = 0; i < 6; ++i)
    twist_error_(i) = pid_[i].update(twist_error_(i), dt.toSec());

  // Get jacobian
  jac_solver_->JntToJac(jnt_pos_, jacobian_);

  // Convert wrench to joint efforts
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    jnt_delta_(i) = 0.0;
    for (unsigned int j = 0; j < 6; ++j)
      jnt_delta_(i) += (jacobian_(j,i) * twist_error_(j));
  }

  // Actually update joints
  for (size_t j = 0; j < joints_.size(); ++j)
    joints_[j]->setPosition(jnt_delta_(j) + joints_[j]->getPosition(), 0.0, 0.0);
}

KDL::Frame CartesianPoseController::getPose()
{
  for (size_t i = 0; i < joints_.size(); ++i)
    jnt_pos_(i) = joints_[i]->getPosition();

  KDL::Frame result;
  jnt_to_pose_solver_->JntToCart(jnt_pos_, result);

  return result;
}

void CartesianPoseController::command(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  // Need to initialize KDL structs
  if (!initialized_)
  {
    ROS_ERROR("CartesianPoseController: Cannot accept goal, controller is not initialized.");
    return;
  }

  // Need transform
  if (!tf_.waitForTransform(goal->header.frame_id, root_link_,
                            goal->header.stamp, ros::Duration(0.1)))
  {
    ROS_ERROR_STREAM("CartesianPoseController: Unable to transform goal to " << root_link_ << ".");
    return;
  }

  // Update last command time before trying to start controller
  last_command_ = ros::Time::now();

  // Try to start up
  if (manager_->requestStart(getName()) != 0)
  {
    ROS_ERROR("CartesianPoseController: Cannot start, blocked by another controller.");
    return;
  }

  tf::Stamped<tf::Pose> stamped;
  tf::poseStampedMsgToTF(*goal, stamped);

  tf_.transformPose(root_link_, stamped, stamped);
  tf::poseTFToKDL(stamped, desired_pose_);
}

std::vector<std::string> CartesianPoseController::getCommandedNames()
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

std::vector<std::string> CartesianPoseController::getClaimedNames()
{
  // Commanded == claimed
  return getCommandedNames();
}


}  // namespace robot_controllers
