/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Unbounded Robotics, Willow Garage nor the names
 *     of their contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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

// Portions are based on splines.h of arm_navigation
// Author: Michael Ferguson, Mrinal Kalakrishnan (splines.h)

#ifndef ROBOT_CONTROLLERS_TRAJECTORY_SPLINE_SAMPLER_H_
#define ROBOT_CONTROLLERS_TRAJECTORY_SPLINE_SAMPLER_H_

#include <robot_controllers/trajectory.h>

namespace robot_controllers
{

/** @brief Spline */
struct Spline
{
  double coef[6];
};

enum SplineType
{
  QUINTIC,
  CUBIC,
  LINEAR
};

/** @brief Helper function for splines. */
static inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; i++)
  {
    powers[i] = powers[i-1]*x;
  }
}

/**
 *  @brief Create a quintic spline with between (p0,v0,a0) and (p1,v1,a1)
 *  @param p0 Starting position.
 *  @param v0 Starting velocity.
 *  @param a0 Starting acceleration.
 *  @param p1 Ending position.
 *  @param v1 Ending velocity.
 *  @param a1 Ending acceleration.
 *  @param t Time between start and end.
 *  @param s The spline
 */
static void QuinticSpline(double p0, double v0, double a0,
                          double p1, double v1, double a1,
                          double t, Spline& s)
{
  if (t == 0.0)
  {
    s.coef[0] = p1;
    s.coef[1] = v1;
    s.coef[2] = 0.5*a1;
    s.coef[3] = 0.0;
    s.coef[4] = 0.0;
    s.coef[5] = 0.0;
  }
  else
  {
    double T[6];
    generatePowers(5, t, T);

    s.coef[0] = p0;
    s.coef[1] = v0;
    s.coef[2] = 0.5*a0;
    s.coef[3] = (-20.0*p0 + 20.0*p1 - 3.0*a0*T[2] + a1*T[2] -
                 12.0*v0*T[1] - 8.0*v1*T[1]) / (2.0*T[3]);
    s.coef[4] = (30.0*p0 - 30.0*p1 + 3.0*a0*T[2] - 2.0*a1*T[2] +
                 16.0*v0*T[1] + 14.0*v1*T[1]) / (2.0*T[4]);
    s.coef[5] = (-12.0*p0 + 12.0*p1 - a0*T[2] + a1*T[2] -
                 6.0*v0*T[1] - 6.0*v1*T[1]) / (2.0*T[5]);
  }
}

/**
 *  @brief Sample from the spline at time t.
 */
static void sampleQuinticSpline(Spline& s, double t,
                                double& position, double& velocity, double& acceleration)
{
  double T[6];
  generatePowers(5, t, T);

  position = T[0]*s.coef[0] +
             T[1]*s.coef[1] +
             T[2]*s.coef[2] +
             T[3]*s.coef[3] +
             T[4]*s.coef[4] +
             T[5]*s.coef[5];

  velocity = T[0]*s.coef[1] +
         2.0*T[1]*s.coef[2] +
         3.0*T[2]*s.coef[3] +
         4.0*T[3]*s.coef[4] +
         5.0*T[4]*s.coef[5];

  acceleration = 2.0*T[0]*s.coef[2] +
                 6.0*T[1]*s.coef[3] +
                12.0*T[2]*s.coef[4] +
                20.0*T[3]*s.coef[5];
}

/**
 *  @brief Constructor for a cubic spline with between (p0,v0) and (p1,v1)
 *  @param p0 Starting position.
 *  @param v0 Starting velocity.
 *  @param p1 Ending position.
 *  @param v1 Ending velocity.
 *  @param t Time between start and end.
 *  @param s Reference to the spline to create.
 */
static void CubicSpline(double p0, double v0, double p1, double v1, double t, Spline& s)
{
  if (t == 0.0)
  {
    s.coef[0] = p1;
    s.coef[1] = v1;
    s.coef[2] = 0.0;
    s.coef[3] = 0.0;
  }
  else
  {
    double T[4];
    generatePowers(3, t, T);

    s.coef[0] = p0;
    s.coef[1] = v0;
    s.coef[2] = (3.0 * p1 - 3.0 * p0 - 2.0 *v0*T[1] - v1*T[1]) / T[2];
    s.coef[3] = (-2.0 * p1 + 2.0 * p0 + v0*T[1] + v1*T[1]) / T[3];
  }
}

/**
 *  @brief Sample from the spline at time t.
 */
static void sampleCubicSpline(Spline s, double t, double& position, double& velocity)
{
  double T[4];
  generatePowers(3, t, T);

  position = T[0]*s.coef[0] +
             T[1]*s.coef[1] +
             T[2]*s.coef[2] +
             T[3]*s.coef[3];

  velocity = T[0]*s.coef[1] +
         2.0*T[1]*s.coef[2] +
         3.0*T[2]*s.coef[3];
}

static void LinearSpline(double p0, double p1, double t, Spline& s)
{
  s.coef[0] = p0;
  s.coef[1] = p1 - p0;
  s.coef[2] = t;
}

static void sampleLinearSpline(Spline& s, double t, double& position)
{
  position = s.coef[0] + (s.coef[1] * t/s.coef[2]);
}

/**
 *  @brief Sampler that uses splines
 */
class SplineTrajectorySampler : public TrajectorySampler
{
  struct Segment
  {
    double start_time;
    double end_time;
    int type;  /// spline type (to choose interpolation)
    std::vector<Spline> splines;  /// one spline per joint
  };

public:
  /** @brief Construct a trajectory sampler */
  SplineTrajectorySampler(const Trajectory& trajectory) :
    trajectory_(trajectory)
  {
    // Check for invalid trajectory size
    if (trajectory.size() == 0)
    {
      throw 0;
    }

    // Check for number of joints
    size_t num_joints = trajectory.points[0].q.size();

    // Trajectory length one is special
    if (trajectory.size() == 1)
    {
      segments_.resize(1);
      segments_[0].start_time = segments_[0].end_time = trajectory.points[0].time;

      // Set up spline
      segments_[0].splines.resize(num_joints);
      for (size_t j = 0; j < num_joints; ++j)
      {
        LinearSpline(trajectory.points[0].q[j],
                     trajectory.points[0].q[j],
                     0.0,
                     segments_[0].splines[j]);
      }
      segments_[0].type = LINEAR;
      result.q.resize(num_joints);
    }
    else
    {
      // We put a segment in between each pair of points
      segments_.resize(trajectory.size()-1);

      // Setup up the segments
      for (size_t p = 0; p < segments_.size(); ++p)
      {
        // This segment is from p to p+1
        segments_[p].start_time = trajectory.points[p].time;
        segments_[p].end_time = trajectory.points[p+1].time;

        // Set up spline
        segments_[p].splines.resize(num_joints);
        if (trajectory.points[p].qdd.size() == trajectory.points[p].q.size())
        {
          result.q.resize(num_joints);
          result.qd.resize(num_joints);
          result.qdd.resize(num_joints);

          // Have accelerations, will use Quintic.
          for (size_t j = 0; j < num_joints; ++j)
          {
            QuinticSpline(trajectory.points[p].q[j],
                          trajectory.points[p].qd[j],
                          trajectory.points[p].qdd[j],
                          trajectory.points[p+1].q[j],
                          trajectory.points[p+1].qd[j],
                          trajectory.points[p+1].qdd[j],
                          segments_[p].end_time - segments_[p].start_time,
                          segments_[p].splines[j]);
          }
          segments_[p].type = QUINTIC;
        }
        else if (trajectory.points[p].qd.size() == trajectory.points[p].q.size())
        {
          result.q.resize(num_joints);
          result.qd.resize(num_joints);

          // Velocities + Positions, do Cubic.
          for (size_t j = 0; j < num_joints; ++j)
          {
            CubicSpline(trajectory.points[p].q[j],
                        trajectory.points[p].qd[j],
                        trajectory.points[p+1].q[j],
                        trajectory.points[p+1].qd[j],
                        segments_[p].end_time - segments_[p].start_time,
                        segments_[p].splines[j]);
          }
          segments_[p].type = CUBIC;
        }
        else
        {
          result.q.resize(num_joints);
 
          // Lame -- only positions do linear
          for (size_t j = 0; j < num_joints; ++j)
          {
            LinearSpline(trajectory.points[p].q[j],
                         trajectory.points[p+1].q[j],
                         segments_[p].end_time - segments_[p].start_time,
                         segments_[p].splines[j]);
          }
          segments_[p].type = LINEAR;
        }
      }
    }
    seg_ = -1;
  }

  /** @brief Sample from this trajectory */
  virtual TrajectoryPoint sample(double time)
  {
    // Which segment to sample from.
    while ((seg_ + 1 < int(segments_.size())) &&
           (segments_[seg_ + 1].start_time < time))
    {
      ++seg_;
    }

    // Check beginning of trajectory, return empty trajectory point if not started.
    if (seg_ == -1)
      return TrajectoryPoint();

    // Check end of trajectory, restrict time.
    if (time > end_time())
      time = end_time();

    // Sample segment for each joint.
    for (size_t i = 0; i < segments_[seg_].splines.size(); ++i)
    {
      if (segments_[seg_].type == QUINTIC)
      {
        sampleQuinticSpline(segments_[seg_].splines[i], time - segments_[seg_].start_time,
                            result.q[i], result.qd[i], result.qdd[i]);
      }
      else if(segments_[seg_].type == CUBIC)
      {
        sampleCubicSpline(segments_[seg_].splines[i], time - segments_[seg_].start_time,
                          result.q[i], result.qd[i]);
      }
      else
      {
        sampleLinearSpline(segments_[seg_].splines[i], time - segments_[seg_].start_time,
                           result.q[i]);
      }
    }

    result.time = time;
    return result;
  }

  /** @brief Get the end time of our trajectory */
  virtual double end_time()
  {
    return segments_[segments_.size()-1].end_time;
  }

  /** @brief Get the trajectory that we are sampling from. */
  virtual Trajectory getTrajectory()
  {
    return trajectory_;
  }

private:
  std::vector<Segment> segments_;
  Trajectory trajectory_;
  TrajectoryPoint result;
  int seg_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_TRAJECTORY_SPLINE_SAMPLER_H_
