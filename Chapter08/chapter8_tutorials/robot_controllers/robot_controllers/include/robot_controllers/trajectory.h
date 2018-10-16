/*********************************************************************
 * Software License Agreement (BSD License)
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

// Author: Michael Ferguson

#ifndef ROBOT_CONTROLLERS_TRAJECTORY_H_
#define ROBOT_CONTROLLERS_TRAJECTORY_H_

#include <ros/ros.h>
#include <angles/angles.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace robot_controllers
{

/**
 *  @brief Basis for a Trajectory Point
 */
struct TrajectoryPoint
{
  std::vector<double> q;
  std::vector<double> qd;
  std::vector<double> qdd;
  double time;
};

struct Trajectory
{
  std::vector<TrajectoryPoint> points;

  size_t size() const
  {
    return points.size();
  }
};

/**
 *  @brief Convert message into Trajectory.
 *  @param message The trajectory message.
 *  @param joints Vector of joint names, used to order the values within a TrajectoryPoint
 *  @param trajectory The returned trajectory
 *  @returns True if trajectory generated, false otherwise.
 */
inline bool trajectoryFromMsg(const trajectory_msgs::JointTrajectory& message,
                              const std::vector<std::string> joints,
                              Trajectory* trajectory)
{
  // Find mapping of joint names into message joint names
  std::vector<int> mapping(joints.size(), -1);
  for (size_t j = 0; j < joints.size(); ++j)
  {
    for (size_t i = 0; i < message.joint_names.size(); ++i)
    {
      if (joints[j] == message.joint_names[i])
      {
        mapping[j] = i;
        break;
      }
    }
    if (mapping[j] == -1)
    {
      return false;
    }
  }

  // Make sure trajectory is empty
  trajectory->points.clear();

  double start_time = message.header.stamp.toSec();
  if (start_time == 0.0)
    start_time = ros::Time::now().toSec();

  // Fill in Trajectory
  for (size_t p = 0; p < message.points.size(); ++p)
  {
    TrajectoryPoint point;
    for (size_t j = 0; j < joints.size(); ++j)
    {
      point.q.push_back(message.points[p].positions[mapping[j]]);
      if (message.points[p].velocities.size() == message.points[p].positions.size())
        point.qd.push_back(message.points[p].velocities[mapping[j]]);
      if (message.points[p].accelerations.size() == message.points[p].positions.size())
        point.qdd.push_back(message.points[p].accelerations[mapping[j]]);
    }
    point.time = start_time + message.points[p].time_from_start.toSec(); 
    trajectory->points.push_back(point);
  }

  // Parsed this
  return true;
}

/**
 *  @brief Splice two trajectories.
 *  @param t1 First trajectory to splice
 *  @param t2 Second trajectory to splice (will overwrite first)
 *  @param time The start time for the new trajectory (typically, current system time)
 *  @param t The new trajectory.
 */
inline bool spliceTrajectories(const Trajectory& t1,
                               const Trajectory& t2,
                               const double time,
                               Trajectory * t)
{
  // Need at least one point in t1 for the following code to work
  if (t1.size() == 0)
  {
    return false;
  }

  // Need at least one point in t2 for the following code to work
  if (t2.size() == 0)
  {
    *t = t1;
    return true;
  }

  // Check sizes
  size_t num_joints = t1.points[0].q.size();
  bool has_velocities = (t1.points[0].qd.size() == num_joints) &&
                        (t2.points[0].qd.size() == num_joints);
  bool has_accelerations = (t1.points[0].qdd.size() == num_joints) &&
                           (t2.points[0].qdd.size() == num_joints);

  // Just to be sure
  t->points.clear();

  // When does t2 start?
  double start_t2 = t2.points[0].time;

  // Find points in t1 after time, but before start_t2
  for (size_t p = 0; p < t1.size(); ++p)
  {
    if (t1.points[p].time >= time && t1.points[p].time < start_t2)
    {
      if (t1.points[p].time > time && t->size() == 0)
      {
        /* This is first point in trajectory, and is after our
           start time, see if we can add one point in front of it */
        if (p > 0)
         t->points.push_back(t1.points[p-1]);
      }
      t->points.push_back(t1.points[p]);
    }
  }

  // Add points from t2
  for (size_t p = 0; p < t2.size(); ++p)
  {
    if (t2.points[p].time >= time)
    {
      if (t2.points[p].time > time && t->size() == 0)
      {
        /* This is first point in trajectory, and is after our
           start time, see if we can add one point in front of it */
        if (p > 0)
         t->points.push_back(t2.points[p-1]);
        else if (t1.size() > 0)
         t->points.push_back(t1.points[t1.size()-1]);
      }
      t->points.push_back(t2.points[p]);
    }
  }

  if (!has_accelerations)
  {
    // Remove any accelerations in output trajectory
    for (size_t i = 0; i < t->points.size(); i++)
    {
      t->points[i].qdd.clear();
    }
  }

  if (!has_velocities)
  {
    // Remove any velocities in output trajectory
    for (size_t i = 0; i < t->points.size(); i++)
    {
      t->points[i].qd.clear();
    }
  }

  return true;
}

/**
 *  @brief Print trajectory to ROS INFO
 */
inline void rosPrintTrajectory(Trajectory& t)
{
  ROS_INFO_STREAM("Trajectory with " << t.size() << " points:");
  for (size_t p = 0; p < t.size(); ++p)
  {
    ROS_INFO_STREAM("  Point " << p << " at " << std::setprecision (15) << t.points[p].time);
    for (size_t j = 0; j < t.points[p].q.size(); ++j)
    {
      if (t.points[p].qdd.size() == t.points[p].q.size())
      {
        ROS_INFO_STREAM("    " << std::setprecision (5) << t.points[p].q[j] <<
                          ", " << std::setprecision (5) << t.points[p].qd[j] <<
                          ", " << std::setprecision (5) << t.points[p].qdd[j]);
      }
      else if(t.points[p].q.size() == t.points[p].q.size())
      {
        ROS_INFO_STREAM("    " << std::setprecision (5) << t.points[p].q[j] <<
                          ", " << std::setprecision (5) << t.points[p].qd[j]);
      }
      else
      {
        ROS_INFO_STREAM("    " << std::setprecision (5) << t.points[p].q[j]);
      }
    }
  }
}

/**
 * @brief Windup the trajectory so that continuous joints do not wrap
 * @param continuous Which joints in each TrajectoryPoint are continuous.
 * @param trajectory The trajectory to unwind.
 */
inline bool windupTrajectory(std::vector<bool> continuous,
                             Trajectory& trajectory)
{
  for (size_t p = 0; p < trajectory.size(); p++)
  {
    if (continuous.size() != trajectory.points[p].q.size())
    {
      // Size does not match
      return false;
    }

    for (size_t j = 0; j < continuous.size(); j++)
    {
      if (continuous[j])
      {
        if (p > 0)
        {
          // Unwind by taking shortest path from previous point
          double shortest = angles::shortest_angular_distance(trajectory.points[p-1].q[j], trajectory.points[p].q[j]);
          trajectory.points[p].q[j]  =  trajectory.points[p-1].q[j] + shortest;
        }
        else
        {
          // Start between -PI and PI
          trajectory.points[p].q[j] = angles::normalize_angle(trajectory.points[p].q[j]);
        }
      }
    }
  }
  return true;
}

inline bool unwindTrajectoryPoint(std::vector<bool> continuous,
                                  TrajectoryPoint& p)
{
  if (continuous.size() != p.q.size())
  {
    return false;
  }

  for (size_t j = 0; j < continuous.size(); j++)
  {
    if (continuous[j])
    {
      p.q[j] = angles::normalize_angle(p.q[j]);
    }
  }

  return true;
}

/**
 *  @brief Base class for samplers of trajectories.
 */
class TrajectorySampler
{
public:
  /** @brief Construct a trajectory sampler */
  TrajectorySampler() {}
  virtual ~TrajectorySampler() {}

  /** @brief Sample from this trajectory */
  virtual TrajectoryPoint sample(double time) = 0;

  /** @brief Get the end time of our trajectory */
  virtual double end_time() = 0;

  /** @brief Get the trajectory that we are sampling from. */
  virtual Trajectory getTrajectory() = 0;

private:
  // You no copy...
  TrajectorySampler(const TrajectorySampler&);
  TrajectorySampler& operator=(const TrajectorySampler&);
};

}  // namespace robot_controllers

#endif // ROBOT_CONTROLLERS_TRAJECTORY_H_
