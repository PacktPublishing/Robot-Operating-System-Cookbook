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

#include <pluginlib/class_list_macros.h>
#include <robot_controllers/point_head.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

PLUGINLIB_EXPORT_CLASS(robot_controllers::PointHeadController, robot_controllers::Controller)

namespace robot_controllers
{

int PointHeadController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  Controller::init(nh, manager);
  manager_ = manager;

  // No initial sampler
  boost::mutex::scoped_lock lock(sampler_mutex_);
  sampler_.reset();
  preempted_ = false;

  // Get parameters
  nh.param<bool>("stop_with_action", stop_with_action_, false);

  // Setup Joints */
  head_pan_ = manager_->getJointHandle("head_pan_joint");
  head_tilt_ = manager_->getJointHandle("head_tilt_joint");

  // Parse UDRF/KDL
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR_NAMED("PointHeadController",
      "Failed to parse URDF, is robot_description parameter set?");
    return -1;
  }

  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
  {
    ROS_ERROR_NAMED("PointHeadController", "Failed to construct KDL tree");
    return -1;
  }

  // Find parent of pan joint
  KDL::SegmentMap segment_map = kdl_tree_.getSegments();
  for (KDL::SegmentMap::iterator it = segment_map.begin();
       it != segment_map.end();
       ++it)
  {
#ifdef KDL_USE_NEW_TREE_INTERFACE
    if (it->second->segment.getJoint().getName() == head_pan_->getName())
      root_link_ = it->second->parent->first;
#else
    if (it->second.segment.getJoint().getName() == head_pan_->getName())
      root_link_ = it->second.parent->first;
#endif
  }

  // Start action server
  server_.reset(new head_server_t(nh, "",
                boost::bind(&PointHeadController::executeCb, this, _1),
                false));
  server_->start();

  initialized_ = true;
  return 0;
}

bool PointHeadController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("PointHeadController",
                    "Unable to start, not initialized.");
    return false;
  }

  if (!server_->isActive())
  {
    ROS_ERROR_NAMED("PointHeadController",
                    "Unable to start, action server is not active.");
    return false;
  }

  return true;
}

bool PointHeadController::stop(bool force)
{
  if (!initialized_)
    return true;

  if (server_->isActive())
  {
    if (force)
    {
      // Shut down the action
      server_->setAborted(result_, "Controller manager forced preemption.");
      ROS_DEBUG_NAMED("PointHeadController",
                      "Controller manager forced preemption.");
      return true;
    }

    // Do not abort unless forced
    return false;
  }

  // Just holding position, go ahead and stop us
  return true;
}

bool PointHeadController::reset()
{
  stop(true);  // force stop ourselves
  return (manager_->requestStop(getName()) == 0);
}

void PointHeadController::update(const ros::Time& now, const ros::Duration& dt)
{
  if (!initialized_)
    return;

  // We have a trajectory to execute?
  if (server_->isActive() && sampler_)
  {
    boost::mutex::scoped_lock lock(sampler_mutex_);

    // Interpolate trajectory
    TrajectoryPoint p = sampler_->sample(now.toSec());
    last_sample_ = p;

    // Are we done?
    if (now.toSec() > sampler_->end_time())
      server_->setSucceeded(result_, "OK");

    // Send trajectory to joints
    if (p.q.size() == 2)
    {
      head_pan_->setPosition(p.q[0], p.qd[0], 0.0);
      head_tilt_->setPosition(p.q[1], p.qd[1], 0.0);
    }
  }
  else if (last_sample_.q.size() == 2)
  {
    // Hold Position
    head_pan_->setPosition(last_sample_.q[0], 0.0, 0.0);
    head_tilt_->setPosition(last_sample_.q[1], 0.0, 0.0);
  }
}

void PointHeadController::executeCb(const control_msgs::PointHeadGoalConstPtr& goal)
{
  float head_pan_goal_ = 0.0;
  float head_tilt_goal_ = 0.0;
  try
  {
    geometry_msgs::PointStamped target_in_pan, target_in_tilt;
    listener_.transformPoint(root_link_, ros::Time(0), goal->target, goal->target.header.frame_id, target_in_pan);
    listener_.transformPoint("head_pan_link", ros::Time(0), goal->target, goal->target.header.frame_id, target_in_tilt);
    head_pan_goal_ = atan2(target_in_pan.point.y, target_in_pan.point.x);
    head_tilt_goal_ = -atan2(target_in_tilt.point.z, sqrt(pow(target_in_tilt.point.x, 2) + pow(target_in_tilt.point.y, 2)));
  }
  catch(const tf::TransformException &ex)
  {
    server_->setAborted(result_, "Could not transform goal.");
    ROS_WARN_NAMED("PointHeadController", "Could not transform goal.");
    return;
  }

  // Turn goal into a trajectory
  Trajectory t;
  t.points.resize(2);
  if (preempted_)
  {
    // Starting point is last sample
    t.points[0] = last_sample_;
  }
  else
  {
    // Starting point is current position, assume not moving
    t.points[0].q.push_back(head_pan_->getPosition());
    t.points[0].q.push_back(head_tilt_->getPosition());
    t.points[0].qd.push_back(0.0);
    t.points[0].qd.push_back(0.0);
    t.points[0].qdd.push_back(0.0);
    t.points[0].qdd.push_back(0.0);
    t.points[0].time = ros::Time::now().toSec();
  }

  // Ending point is goal position, not moving
  t.points[1].q.push_back(head_pan_goal_);
  t.points[1].q.push_back(head_tilt_goal_);
  t.points[1].qd.push_back(0.0);
  t.points[1].qd.push_back(0.0);
  t.points[1].qdd.push_back(0.0);
  t.points[1].qdd.push_back(0.0);

  // Determine how long to take to execute this trajectory
  double max_pan_vel = head_pan_->getVelocityMax();
  double max_tilt_vel = head_tilt_->getVelocityMax();
  if (goal->max_velocity > 0.0)
  {
    max_pan_vel = fmin(goal->max_velocity, head_pan_->getVelocityMax());
    max_tilt_vel = fmin(goal->max_velocity, head_tilt_->getVelocityMax());
  }
  double pan_transit = fabs((t.points[1].q[0] - t.points[0].q[0]) / max_pan_vel);
  double tilt_transit = fabs((t.points[1].q[1] - t.points[0].q[1]) / max_tilt_vel);
  t.points[1].time = t.points[0].time + fmax(fmax(pan_transit, tilt_transit), goal->min_duration.toSec());

  {
    boost::mutex::scoped_lock lock(sampler_mutex_);
    sampler_.reset(new SplineTrajectorySampler(t));
  }

  if (manager_->requestStart(getName()) != 0)
  {
    server_->setAborted(result_, "Cannot point head, unable to start controller.");
    ROS_ERROR_NAMED("PointHeadController",
                    "Cannot point head, unable to start controller.");
    return;
  }

  preempted_ = false;
  while (server_->isActive())
  {
    if (server_->isPreemptRequested())
    {
      server_->setPreempted(result_, "Pointing of the head has been preempted");
      ROS_DEBUG_NAMED("PointHeadController",
                      "Pointing of the head has been preempted");
      preempted_ = true;
      break;
    }

    // No feedback needed for PointHeadAction
    ros::Duration(1/50.0).sleep();
  }

  // Stop this controller if desired (and not preempted)
  if (stop_with_action_ && !preempted_)
    manager_->requestStop(getName());

  ROS_DEBUG_NAMED("PointHeadController", "Done pointing head");
}

std::vector<std::string> PointHeadController::getCommandedNames()
{
  std::vector<std::string> names;
  names.push_back(head_pan_->getName());
  names.push_back(head_tilt_->getName());
  return names;
}

std::vector<std::string> PointHeadController::getClaimedNames()
{
  // Claimed == commanded.
  return getCommandedNames();
}

}  // namespace robot_controllers
