/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2015, Fetch Robotics Inc.
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
#include <robot_controllers/follow_joint_trajectory.h>

using angles::shortest_angular_distance;

PLUGINLIB_EXPORT_CLASS(robot_controllers::FollowJointTrajectoryController, robot_controllers::Controller)

namespace robot_controllers
{

FollowJointTrajectoryController::FollowJointTrajectoryController() :
    initialized_(false)
{
}

int FollowJointTrajectoryController::init(ros::NodeHandle& nh, ControllerManager* manager)
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

  // Get Joint Names
  joint_names_.clear();
  XmlRpc::XmlRpcValue names;
  if (!nh.getParam("joints", names))
  {
    ROS_ERROR_STREAM("No joints given for " << nh.getNamespace());
    return -1;
  }
  if (names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Joints not in a list for " << nh.getNamespace());
    return -1;
  }
  for (int i = 0; i < names.size(); ++i)
  {
    XmlRpc::XmlRpcValue &name_value = names[i];
    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("Not all joint names are strings for " << nh.getNamespace());
      return -1;
    }
    joint_names_.push_back(static_cast<std::string>(name_value));
  }

  // Get parameters
  nh.param<bool>("stop_with_action", stop_with_action_, false);
  nh.param<bool>("stop_on_path_violation", stop_on_path_violation_, false);

  // Get Joint Handles, setup feedback
  joints_.clear();
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    JointHandlePtr j = manager_->getJointHandle(joint_names_[i]);
    feedback_.joint_names.push_back(j->getName());
    joints_.push_back(j);
    continuous_.push_back(j->isContinuous());
  }

  // Update feedback
  feedback_.desired.positions.resize(joints_.size());
  feedback_.desired.velocities.resize(joints_.size());
  feedback_.desired.accelerations.resize(joints_.size());
  feedback_.actual.positions.resize(joints_.size());
  feedback_.actual.velocities.resize(joints_.size());
  feedback_.actual.effort.resize(joints_.size());
  feedback_.error.positions.resize(joints_.size());
  feedback_.error.velocities.resize(joints_.size());

  // Update tolerances
  path_tolerance_.q.resize(joints_.size());
  path_tolerance_.qd.resize(joints_.size());
  path_tolerance_.qdd.resize(joints_.size());
  goal_tolerance_.q.resize(joints_.size());
  goal_tolerance_.qd.resize(joints_.size());
  goal_tolerance_.qdd.resize(joints_.size()); 

  // Setup ROS interfaces
  server_.reset(new server_t(nh, "",
                             boost::bind(&FollowJointTrajectoryController::executeCb, this, _1),
                             false));
  server_->start();

  initialized_ = true;
  return 0;
}

bool FollowJointTrajectoryController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("FollowJointTrajectoryController",
                    "Unable to start, not initialized.");
    return false;
  }

  if (!server_->isActive())
  {
    ROS_ERROR_NAMED("FollowJointTrajectoryController",
                    "Unable to start, action server is not active.");
    return false;
  }

  return true;
}

bool FollowJointTrajectoryController::stop(bool force)
{
  if (!initialized_)
    return true;

  if (server_->isActive())
  {
    if (force)
    {
      // Shut down the action
      control_msgs::FollowJointTrajectoryResult result;
      server_->setAborted(result, "Controller manager forced preemption.");
      return true;
    }
    // Do not abort unless forced
    return false;
  }

  // Just holding position, go ahead and stop us
  return true;
}

bool FollowJointTrajectoryController::reset()
{
  stop(true);  // force stop ourselves
  return (manager_->requestStop(getName()) == 0);
}

void FollowJointTrajectoryController::update(const ros::Time& now, const ros::Duration& dt)
{
  if (!initialized_)
    return;

  // Is trajectory active?
  if (server_->isActive() && sampler_)
  {
    boost::mutex::scoped_lock lock(sampler_mutex_);

    // Interpolate trajectory
    TrajectoryPoint p = sampler_->sample(now.toSec());
    unwindTrajectoryPoint(continuous_, p);
    last_sample_ = p;

    // Update joints
    if (p.q.size() == joints_.size())
    {
      // Position is good
      for (size_t i = 0; i < joints_.size(); ++i)
      {
        feedback_.desired.positions[i] = p.q[i];
      }

      if (p.qd.size() == joints_.size())
      {
        // Velocity is good
        for (size_t i = 0; i < joints_.size(); ++i)
        {
          feedback_.desired.velocities[i] = p.qd[i];
        }

        if (p.qdd.size() == joints_.size())
        {
          // Acceleration is good
          for (size_t i = 0; i < joints_.size(); ++i)
          {
            feedback_.desired.accelerations[i] = p.qdd[i];
          }
        }
      }

      // Fill in actual
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        feedback_.actual.positions[j] = joints_[j]->getPosition();
        feedback_.actual.velocities[j] = joints_[j]->getVelocity();
        feedback_.actual.effort[j] = joints_[j]->getEffort();
      }

      // Fill in error
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        feedback_.error.positions[j] = shortest_angular_distance(feedback_.desired.positions[j],
                                                                 feedback_.actual.positions[j]);
        feedback_.error.velocities[j] = feedback_.actual.velocities[j] -
                                        feedback_.desired.velocities[j];
      }

      // Check that we are within path tolerance
      if (has_path_tolerance_)
      {
        for (size_t j = 0; j < joints_.size(); ++j)
        {
          if ((path_tolerance_.q[j] > 0) &&
              (fabs(feedback_.error.positions[j]) > path_tolerance_.q[j]))
          {
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            server_->setAborted(result, "Trajectory path tolerances violated (position).");
            ROS_ERROR("Trajectory path tolerances violated (position).");
            if (stop_on_path_violation_)
            {
              manager_->requestStop(getName());
            }
            break;
          }

          if ((path_tolerance_.qd[j] > 0) &&
              (fabs(feedback_.error.velocities[j]) > path_tolerance_.qd[j]))
          {
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            server_->setAborted(result, "Trajectory path tolerances violated (velocity).");
            ROS_ERROR("Trajectory path tolerances violated (velocity).");
            if (stop_on_path_violation_)
            {
              manager_->requestStop(getName());
            }
            break;
          }
        }
      }

      // Check that we are within goal tolerance
      if (now.toSec() >= sampler_->end_time())
      {
        bool inside_tolerances = true;
        for (size_t j = 0; j < joints_.size(); ++j)
        {
          if ((goal_tolerance_.q[j] > 0) &&
              (fabs(feedback_.error.positions[j]) > goal_tolerance_.q[j]))
          {
            inside_tolerances = false;
          }
        }

        if (inside_tolerances)
        {
          control_msgs::FollowJointTrajectoryResult result;
          result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
          server_->setSucceeded(result, "Trajectory succeeded.");
          ROS_DEBUG("Trajectory succeeded");
        }
        else if (now.toSec() > (sampler_->end_time() + goal_time_tolerance_ + 0.6))  // 0.6s matches PR2
        {
          control_msgs::FollowJointTrajectoryResult result;
          result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
          server_->setAborted(result, "Trajectory not executed within time limits.");
          ROS_ERROR("Trajectory not executed within time limits");
        }
      }

      // Update joints
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        joints_[j]->setPosition(feedback_.desired.positions[j],
                                feedback_.desired.velocities[j],
                                0.0);
      }
    }
  }
  else if (last_sample_.q.size() == joints_.size())
  {
    // Hold Position Unless Path Tolerance Violated
    if (has_path_tolerance_ && stop_on_path_violation_)
    {
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        if ((path_tolerance_.q[j] > 0) &&
            (fabs(joints_[j]->getPosition() - last_sample_.q[j]) > path_tolerance_.q[j]))
        {
          manager_->requestStop(getName());
          break;
        }
      }
    }

    for (size_t j = 0; j < joints_.size(); ++j)
    {
      joints_[j]->setPosition(last_sample_.q[j],
                              0.0,
                              0.0);
    }
  }
}

/*
 * Specification is basically the message:
 * http://ros.org/doc/indigo/api/control_msgs/html/action/FollowJointTrajectory.html
 */
void FollowJointTrajectoryController::executeCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  control_msgs::FollowJointTrajectoryResult result;

  if (!initialized_)
  {
    server_->setAborted(result, "Controller is not initialized.");
    return;
  }

  if (goal->trajectory.points.empty())
  {
    // Stop
    manager_->requestStop(getName());
    return;
  }

  if (goal->trajectory.joint_names.size() != joints_.size())
  {
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    server_->setAborted(result, "Trajectory goal size does not match controlled joints size.");
    ROS_ERROR("Trajectory goal size does not match controlled joints size.");
    return;
  }

  Trajectory new_trajectory;
  Trajectory executable_trajectory;

  // Make a trajectory from our message
  if (!trajectoryFromMsg(goal->trajectory, joint_names_, &new_trajectory))
  {
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    server_->setAborted(result, "Trajectory goal does not match controlled joints");
    ROS_ERROR("Trajectory goal does not match controlled joints");
    return;
  }

  // If preempted, need to splice on things together
  if (preempted_)
  {
    // If the sampler had a large trajectory, we may just be cutting into it
    if (sampler_ && (sampler_->getTrajectory().size() > 2))
    {
      if (!spliceTrajectories(sampler_->getTrajectory(),
                              new_trajectory,
                              ros::Time::now().toSec(),
                              &executable_trajectory))
      {
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        server_->setAborted(result, "Unable to splice trajectory");
        ROS_ERROR("Unable to splice trajectory");
        return;
      }
    }
    else
    {
      // Previous trajectory was only 2 points, use last_sample + new trajectory
      Trajectory t;
      t.points.push_back(last_sample_);
      if (!spliceTrajectories(t,
                              new_trajectory,
                              0.0, /* take all points */
                              &executable_trajectory))
      {
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        server_->setAborted(result, "Unable to splice trajectory");
        ROS_ERROR("Unable to splice trajectory");
        return;
      }
    }
  }
  else
  {
    if (new_trajectory.size() > 1)
    {
      // use the generated trajectory
      executable_trajectory = new_trajectory;

      // if this hasn't started yet, need to insert current position
      if (goal->trajectory.points[0].time_from_start.toSec() > 0.0)
      {
        executable_trajectory.points.insert(
          executable_trajectory.points.begin(),
          getPointFromCurrent(new_trajectory.points[0].qd.size() > 0,
                              new_trajectory.points[0].qdd.size() > 0,
                              true));
      }
    }
    else
    {
      // A single point, with nothing in the queue!
      executable_trajectory.points.push_back(
          getPointFromCurrent(new_trajectory.points[0].qd.size() > 0,
                              new_trajectory.points[0].qdd.size() > 0,
                              true));
      executable_trajectory.points.push_back(new_trajectory.points[0]);
    }
  }

  // Windup executable trajectory so spline is smooth
  windupTrajectory(continuous_, executable_trajectory);

  // Create trajectory sampler
  {
    boost::mutex::scoped_lock lock(sampler_mutex_);
    sampler_.reset(new SplineTrajectorySampler(executable_trajectory));
  }

  // Convert the path tolerances into a more usable form
  if (goal->path_tolerance.size() == joints_.size())
  {
    has_path_tolerance_ = true;
    for (size_t j = 0; j < joints_.size(); ++j)
    {
      // Find corresponding indices between path_tolerance and joints
      int index = -1;
      for (size_t i = 0; i < goal->path_tolerance.size(); ++i)
      {
        if (joints_[j]->getName() == goal->path_tolerance[i].name)
        {
          index = i;
          break;
        }
      }
      if (index == -1)
      {
        // Every joint must have a tolerance and this one does not
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        server_->setAborted(result, "Unable to convert path tolerances");
        ROS_ERROR("Unable to convert path tolerances");
        return;
      }
      path_tolerance_.q[j] = goal->path_tolerance[index].position;
      path_tolerance_.qd[j] = goal->path_tolerance[index].velocity;
      path_tolerance_.qdd[j] = goal->path_tolerance[index].acceleration;
    }
  }
  else
  {
    has_path_tolerance_ = false;
  }

  // Convert the goal tolerances into a more usable form
  if (goal->goal_tolerance.size() == joints_.size())
  {
    for (size_t j = 0; j < joints_.size(); ++j)
    {
      // Find corresponding indices between goal_tolerance and joints
      int index = -1;
      for (size_t i = 0; i < goal->goal_tolerance.size(); ++i)
      {
        if (joints_[j]->getName() == goal->goal_tolerance[i].name)
        {
          index = i;
          break;
        }
      }
      if (index == -1)
      {
        // Every joint must have a tolerance and this one does not
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        server_->setAborted(result, "Unable to convert goal tolerances");
        ROS_ERROR("Unable to convert goal tolerances");
        return;
      }
      goal_tolerance_.q[j] = goal->goal_tolerance[index].position;
      goal_tolerance_.qd[j] = goal->goal_tolerance[index].velocity;
      goal_tolerance_.qdd[j] = goal->goal_tolerance[index].acceleration;
    }
  }
  else
  {
    // Set defaults
    for (size_t j = 0; j < joints_.size(); ++j)
    {
      goal_tolerance_.q[j] = 0.02;  // tolerance is same as PR2
      goal_tolerance_.qd[j] = 0.02;
      goal_tolerance_.qdd[j] = 0.02;
    }
  }
  goal_time_tolerance_ = goal->goal_time_tolerance.toSec();

  ROS_DEBUG("Executing new trajectory");

  if (manager_->requestStart(getName()) != 0)
  {
    result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
    server_->setAborted(result, "Cannot execute trajectory, unable to start controller.");
    ROS_ERROR("Cannot execute trajectory, unable to start controller.");
    return;
  }

  preempted_ = false;
  while (server_->isActive())
  {
    if (server_->isPreemptRequested())
    {
      control_msgs::FollowJointTrajectoryResult result;
      server_->setPreempted(result, "Trajectory preempted");
      ROS_DEBUG("Trajectory preempted");
      preempted_ = true;
      break;
    }

    // Publish feedback
    feedback_.header.stamp = ros::Time::now();
    server_->publishFeedback(feedback_);
    ros::Duration(1/50.0).sleep();
  }

  {
    boost::mutex::scoped_lock lock(sampler_mutex_);
    sampler_.reset();
  }

  // Stop this controller if desired (and not preempted)
  if (stop_with_action_ && !preempted_)
    manager_->requestStop(getName());

  ROS_DEBUG("Done executing trajectory");
}

TrajectoryPoint FollowJointTrajectoryController::getPointFromCurrent(
  bool incl_vel, bool incl_acc, bool zero_vel)
{
  TrajectoryPoint point;

  point.q.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    point.q[j] = joints_[j]->getPosition();

  if (incl_vel && zero_vel)
  {
    point.qd.resize(joints_.size());
    for (size_t j = 0; j < joints_.size(); ++j)
      point.qd[j] = 0.0;
  }
  else if (incl_vel)
  {
    point.qd.resize(joints_.size());
    for (size_t j = 0; j < joints_.size(); ++j)
      point.qd[j] = joints_[j]->getVelocity();
  }

  if (incl_acc)
  {
    point.qdd.resize(joints_.size());
    // Currently no good measure of acceleration, assume none
    for (size_t j = 0; j < joints_.size(); ++j)
      point.qdd[j] = 0.0;
  }

  point.time = ros::Time::now().toSec();

  return point;
}

std::vector<std::string> FollowJointTrajectoryController::getCommandedNames()
{
  return joint_names_;
}

std::vector<std::string> FollowJointTrajectoryController::getClaimedNames()
{
  return joint_names_;
}

}  // namespace robot_controllers
