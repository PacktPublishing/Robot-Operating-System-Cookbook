/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2017, Fetch Robotics Inc.
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

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <robot_controllers/diff_drive_base.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::DiffDriveBaseController, robot_controllers::Controller)

namespace robot_controllers
{

DiffDriveBaseController::DiffDriveBaseController() :
    initialized_(false),
    safety_scaling_(1.0)
{
  theta_ = 0.0;

  odom_.pose.pose.orientation.z = 0.0;
  odom_.pose.pose.orientation.w = 1.0;

  last_sent_x_ = desired_x_ = 0.0;
  last_sent_r_ = desired_r_ = 0.0;

  left_last_timestamp_ = right_last_timestamp_ = 0.0;
  last_command_ = last_update_ = ros::Time(0.0);
}

int DiffDriveBaseController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    ROS_ERROR_NAMED("BaseController", "No controller manager available.");
    initialized_ = false;
    return -1;
  }

  Controller::init(nh, manager);
  manager_ = manager;

  // Initialize joints
  std::string l_name, r_name;
  nh.param<std::string>("l_wheel_joint", l_name, "l_wheel_joint");
  nh.param<std::string>("r_wheel_joint", r_name, "r_wheel_joint");
  left_ = manager_->getJointHandle(l_name);
  right_ = manager_->getJointHandle(r_name);
  if (left_ == NULL || right_ == NULL)
  {
    ROS_ERROR_NAMED("BaseController", "Cannot get wheel joints.");
    initialized_ = false;
    return -1;
  }
  left_last_position_ = left_->getPosition();
  right_last_position_ = right_->getPosition();
  last_update_ = ros::Time::now();

  // Get base parameters
  nh.param<double>("track_width", track_width_, 0.37476);
  nh.param<double>("radians_per_meter", radians_per_meter_, 16.5289);

  // If using an external correction (such as robot_pose_ekf or graft)
  // we should not publish the TF frame from base->odom
  nh.param<bool>("publish_tf", publish_tf_, true);

  // The pose in the odometry message is specified in terms of the odometry frame
  nh.param<std::string>("odometry_frame", odom_.header.frame_id, "odom");

  // The twist in the odometry message is specified in the coordinate frame of the base
  nh.param<std::string>("base_frame", odom_.child_frame_id, "base_link");

  // Get various thresholds below which we supress noise
  nh.param<double>("wheel_rotating_threshold", wheel_rotating_threshold_, 0.001);
  nh.param<double>("rotating_threshold", rotating_threshold_, 0.05);
  nh.param<double>("moving_threshold", moving_threshold_, 0.05);

  double t;
  nh.param<double>("timeout", t, 0.25);
  timeout_ = ros::Duration(t);

  // Get limits of base controller
  nh.param<double>("max_velocity_x", max_velocity_x_, 1.0);
  nh.param<double>("max_velocity_r", max_velocity_r_, 4.5);
  nh.param<double>("max_acceleration_x", max_acceleration_x_, 0.75);
  nh.param<double>("max_acceleration_r", max_acceleration_r_, 3.0);

  // Subscribe to base commands
  cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("command", 1,
                boost::bind(&DiffDriveBaseController::command, this, _1));

  // Publish odometry & tf
  ros::NodeHandle n;
  odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 10);
  if (publish_tf_)
    broadcaster_.reset(new tf::TransformBroadcaster());

  // Publish timer
  double publish_frequency;
  nh.param<double>("publish_frequency", publish_frequency, 100.0);
  odom_timer_ = n.createTimer(ros::Duration(1/publish_frequency),
                              &DiffDriveBaseController::publishCallback,
                              this);

  // Should we use the laser for safety limiting base velocity?
  nh.param<double>("laser_safety_dist", safety_scaling_distance_, 0.0);
  nh.param<double>("robot_safety_width", robot_width_, 0.7);
  if (safety_scaling_distance_ > 0.0)
  {
    scan_sub_ = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1,
                  boost::bind(&DiffDriveBaseController::scanCallback, this, _1));
  }

  initialized_ = true;

  // Should we autostart?
  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

void DiffDriveBaseController::command(const geometry_msgs::TwistConstPtr& msg)
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("BaseController", "Unable to accept command, not initialized.");
    return;
  }

  if (std::isfinite(msg->linear.x) && std::isfinite(msg->angular.z))
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    last_command_ = ros::Time::now();
    desired_x_ = msg->linear.x;
    desired_r_ = msg->angular.z;
  }
  else
  {
    ROS_ERROR_NAMED("BaseController", "Commanded velocities not finite!");
    return;
  }

  manager_->requestStart(getName());
}

bool DiffDriveBaseController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("BaseController", "Unable to start, not initialized.");
    return false;
  }

  return true;
}

bool DiffDriveBaseController::stop(bool force)
{
  // If we have timed out, certainly stop
  if (last_update_ - last_command_ >= timeout_)
    return true;

  // If not moving, stop
  if (last_sent_x_ == 0.0 && last_sent_r_ == 0.0)
    return true;

  // If forced, stop
  if (force)
    return true;

  return false;
}

bool DiffDriveBaseController::reset()
{
  // Reset command
  last_command_ = ros::Time(0);
  return true;
}

void DiffDriveBaseController::update(const ros::Time& now, const ros::Duration& dt)
{
  if (!initialized_)
    return;  // should never really hit this

  // See if we have timed out and need to stop
  if (now - last_command_ >= timeout_)
  {
    ROS_DEBUG_THROTTLE_NAMED(5, "BaseController", "Command timed out.");
    boost::mutex::scoped_lock lock(command_mutex_);
    desired_x_ = desired_r_ = 0.0;
  }

  // Make sure laser has not timed out
  if ((safety_scaling_distance_ > 0.0) &&
      (ros::Time::now() - last_laser_scan_ > ros::Duration(0.5)))
  {
    safety_scaling_ = 0.1;
  }

  // Do velocity acceleration/limiting
  double x, r;
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    // Limit linear velocity based on obstacles
    x = std::max(-max_velocity_x_ * safety_scaling_, std::min(desired_x_, max_velocity_x_ * safety_scaling_));
    // Compute how much we actually scaled the linear velocity
    double actual_scaling = 1.0;
    if (desired_x_ != 0.0)
      actual_scaling = x/desired_x_;
    // Limit angular velocity
    // Scale same amount as linear velocity so that robot still follows the same "curvature"
    r = std::max(-max_velocity_r_, std::min(actual_scaling * desired_r_, max_velocity_r_));
  }
  if (x > last_sent_x_)
  {
    last_sent_x_ += max_acceleration_x_ * (now - last_update_).toSec();
    if (last_sent_x_ > x)
      last_sent_x_ = x;
  }
  else
  {
    last_sent_x_ -= max_acceleration_x_ * (now - last_update_).toSec();
    if (last_sent_x_ < x)
      last_sent_x_ = x;
  }
  if (r > last_sent_r_)
  {
    last_sent_r_ += max_acceleration_r_ * (now - last_update_).toSec();
    if (last_sent_r_ > r)
      last_sent_r_ = r;
  }
  else
  {
    last_sent_r_ -= max_acceleration_r_ * (now - last_update_).toSec();
    if (last_sent_r_ < r)
      last_sent_r_ = r;
  }

  double dx = 0.0;
  double dr = 0.0;

  double left_pos = left_->getPosition();
  double right_pos = right_->getPosition();
  double left_dx = angles::shortest_angular_distance(left_last_position_, left_pos)/radians_per_meter_;
  double right_dx = angles::shortest_angular_distance(right_last_position_, right_pos)/radians_per_meter_;
  double left_vel = static_cast<double>(left_->getVelocity())/radians_per_meter_;
  double right_vel = static_cast<double>(right_->getVelocity())/radians_per_meter_;

  // Threshold the odometry to avoid noise (especially in simulation)
  if (fabs(left_dx) > wheel_rotating_threshold_ ||
      fabs(right_dx) > wheel_rotating_threshold_ ||
      last_sent_x_ != 0.0 ||
      last_sent_r_ != 0.0)
  {
    // Above threshold, update last position
    left_last_position_ = left_pos;
    right_last_position_ = right_pos;
  }
  else
  {
    // Below threshold, zero delta/velocities
    left_dx = right_dx = 0.0;
    left_vel = right_vel = 0.0;
  }

  // Calculate forward and angular differences
  double d = (left_dx+right_dx)/2.0;
  double th = (right_dx-left_dx)/track_width_;

  // Calculate forward and angular velocities
  dx = (left_vel + right_vel)/2.0;
  dr = (right_vel - left_vel)/track_width_;

  // Actually set command
  if (fabs(dx) > moving_threshold_ ||
      fabs(dr) > rotating_threshold_ ||
      last_sent_x_ != 0.0 ||
      last_sent_r_ != 0.0)
  {
    setCommand(last_sent_x_ - (last_sent_r_/2.0 * track_width_),
               last_sent_x_ + (last_sent_r_/2.0 * track_width_));
  }

  // Lock mutex before updating
  boost::mutex::scoped_lock lock(odom_mutex_);

  // Update stored odometry pose...
  theta_ += th/2.0;
  odom_.pose.pose.position.x += d*cos(theta_);
  odom_.pose.pose.position.y += d*sin(theta_);
  theta_ += th/2.0;
  odom_.pose.pose.orientation.z = sin(theta_/2.0);
  odom_.pose.pose.orientation.w = cos(theta_/2.0);
  // ...and twist
  odom_.twist.twist.linear.x = dx;
  odom_.twist.twist.angular.z = dr;

  last_update_ = now;
}

std::vector<std::string> DiffDriveBaseController::getCommandedNames()
{
  std::vector<std::string> names;
  if (left_)
    names.push_back(left_->getName());
  if (right_)
    names.push_back(right_->getName());
  return names;
}

std::vector<std::string> DiffDriveBaseController::getClaimedNames()
{
  // Claimed == Commanded
  return getCommandedNames();
}

void DiffDriveBaseController::publishCallback(const ros::TimerEvent& event)
{
  // Copy message under lock of mutex
  nav_msgs::Odometry msg;
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    msg = odom_;
  }

  // Publish or perish
  msg.header.stamp = ros::Time::now();
  odom_pub_.publish(msg);

  if (publish_tf_)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0));
    transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,
                                         msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z,
                                         msg.pose.pose.orientation.w) );
    /*
     * REP105 (http://ros.org/reps/rep-0105.html)
     *   says: map -> odom -> base_link
     */
    broadcaster_->sendTransform(tf::StampedTransform(transform, msg.header.stamp, msg.header.frame_id, msg.child_frame_id));
  }
}

void DiffDriveBaseController::scanCallback(
  const sensor_msgs::LaserScanConstPtr& scan)
{
  double angle = scan->angle_min;
  double min_dist = safety_scaling_distance_;

  for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment)
  {
    if (std::isfinite(scan->ranges[i]) &&
        scan->ranges[i] > scan->range_min &&
        scan->ranges[i] < min_dist)
    {
      // Only test points in the forward 180 degrees
      if (angle < -1.5 || angle > 1.5)
        continue;

      // Check if point is inside the width of the robot
      double py = sin(angle) * scan->ranges[i];
      if (fabs(py) < (robot_width_/2.0))
        min_dist = scan->ranges[i];
    }
  }

  boost::mutex::scoped_lock lock(command_mutex_);
  safety_scaling_ = std::max(0.1, min_dist / safety_scaling_distance_);
  last_laser_scan_ = ros::Time::now();
}

void DiffDriveBaseController::setCommand(float left, float right)
{
  // Convert meters/sec into radians/sec
  left_->setVelocity(left * radians_per_meter_, 0.0);
  right_->setVelocity(right * radians_per_meter_, 0.0);
}

}  // namespace robot_controllers
