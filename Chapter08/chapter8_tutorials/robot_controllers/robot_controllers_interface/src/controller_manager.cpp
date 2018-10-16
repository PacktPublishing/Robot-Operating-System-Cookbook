/*
 * Copyright (c) 2014-2015, Fetch Robotics Inc.
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

#include <sstream>
#include <robot_controllers_interface/controller_manager.h>

namespace robot_controllers
{

ControllerManager::ControllerManager()
{
}

int ControllerManager::init(ros::NodeHandle& nh)
{
  // Find and load default controllers
  XmlRpc::XmlRpcValue controller_params;
  if (nh.getParam("default_controllers", controller_params))
  {
    if (controller_params.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR_NAMED("ControllerManager", "Parameter 'default_controllers' should be a list.");
      return -1;
    }
    else
    {
      // Load each controller
      for (int c = 0; c < controller_params.size(); c++)
      {
        // Make sure name is valid
        XmlRpc::XmlRpcValue &controller_name = controller_params[c];
        if (controller_name.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_WARN_NAMED("ControllerManager", "Controller name is not a string?");
          continue;
        }

        // Create controller (in a loader)
        load(static_cast<std::string>(controller_name));
      }
    }
  }
  else
  {
    ROS_WARN_NAMED("ControllerManager", "No controllers loaded.");
    return -1;
  }

  // Setup actionlib server
  server_.reset(new server_t(nh, "/query_controller_states",
                             boost::bind(&ControllerManager::execute, this, _1),
                             false));
  server_->start();

  return 0;
}

int ControllerManager::requestStart(const std::string& name)
{
  // Find requested controller
  ControllerLoaderPtr controller;
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    if ((*c)->getController()->getName() == name)
    {
      controller = *c;
      break;
    }
  }

  // Does controller exist?
  if (!controller)
  {
    ROS_ERROR_STREAM_NAMED("ControllerManager", "Unable to start " << name.c_str() << ": no such controller.");
    return -1;
  }

  // Is controller already running?
  if (controller->isActive())
  {
    ROS_DEBUG_STREAM_NAMED("ControllerManager", "Unable to start " << name.c_str() << ": already running.");
    return 0;
  }

  // Check for conflicts
  std::vector<std::string> names = controller->getController()->getCommandedNames();
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    // Only care about active controllers
    if (!(*c)->isActive())
      continue;

    std::vector<std::string> names2 = (*c)->getController()->getClaimedNames();
    bool conflict = false;
    for (size_t i = 0; i < names.size(); i++)
    {
      for (size_t i2 = 0; i2 < names2.size(); i2++)
      {
        if (names[i] == names2[i2])
        {
          conflict = true;
          break;
        }
      }
      if (conflict) break;
    }
    // Have conflict, try to stop controller (without force)
    if (conflict)
    {
      if ((*c)->stop(false))
      {
        ROS_INFO_STREAM_NAMED("ControllerManager", "Stopped " << (*c)->getController()->getName().c_str());
      }
      else
      {
        // Unable to stop c, cannot start controller
        ROS_ERROR_STREAM_NAMED("ControllerManager", "Unable to stop " <<
                                                    (*c)->getController()->getName().c_str() <<
                                                    " when trying to start " << name.c_str());
        return -1;
      }
    }
  }

  // Start controller
  if (controller->start())
  {
    ROS_INFO_STREAM_NAMED("ControllerManager", "Started " << controller->getController()->getName().c_str());
    return 0;
  }

  return -1;  // Unable to start
}

int ControllerManager::requestStop(const std::string& name)
{
  // Find controller
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    if ((*c)->getController()->getName() == name)
    {
      // Stop controller (with force)
      if ((*c)->stop(true))
      {
        ROS_INFO_STREAM_NAMED("ControllerManager", "Stopped " << (*c)->getController()->getName().c_str());
        return 0;
      }
      else
      {
        return -1;  // controller decided not to stop
      }
    }
  }
  return -1;  // no such controller
}

void ControllerManager::update(const ros::Time& time, const ros::Duration& dt)
{
  // Reset handles
  for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
    (*j)->reset();

  // Update controllers
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    (*c)->update(time, dt);
  }
}

void ControllerManager::reset()
{
  // Update controllers
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    (*c)->reset();
  }
}

bool ControllerManager::addJointHandle(JointHandlePtr& j)
{
  // TODO: check for duplicate names?
  joints_.push_back(j);
  return true;
}

HandlePtr ControllerManager::getHandle(const std::string& name)
{
  // Try joints first
  for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
  {
    if ((*j)->getName() == name)
      return *j;
  }

  // Then controllers
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    if ((*c)->getController()->getName() == name)
      return (*c)->getController();
  }

  // Not found
  return HandlePtr();
}

JointHandlePtr ControllerManager::getJointHandle(const std::string& name)
{
  // Try joints first
  for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
  {
    if ((*j)->getName() == name)
      return *j;
  }

  // Not found
  return JointHandlePtr();
}

void ControllerManager::execute(const robot_controllers_msgs::QueryControllerStatesGoalConstPtr& goal)
{
  robot_controllers_msgs::QueryControllerStatesFeedback feedback;
  robot_controllers_msgs::QueryControllerStatesResult result;

  for (size_t i = 0; i < goal->updates.size(); i++)
  {
    // Update this controller
    robot_controllers_msgs::ControllerState state = goal->updates[i];

    // Make sure controller exists
    bool in_controller_list = false;
    for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
    {
      if ((*c)->getController()->getName() == state.name)
      {
        if (state.type != "")
        {
          if (state.type == (*c)->getController()->getType())
          {
            in_controller_list = true;
            break;
          }
          else
          {
            std::stringstream ss;
            ss << "Controller " << state.name << " is of type " << (*c)->getController()->getType() << " not " << state.type;
            getState(result);
            server_->setAborted(result, ss.str());
            return;
          }
        }
        in_controller_list = true;
        break;
      }
    }
    if (!in_controller_list)
    {
      // Check if controller exists on parameter server
      ros::NodeHandle nh;
      if (nh.hasParam(state.name))
      { 
        // Create controller (in a loader)
        if (!load(static_cast<std::string>(state.name)))
        {
          std::stringstream ss;
          ss << "Failed to load controller: " << state.name;
          getState(result);
          server_->setAborted(result, ss.str());
          return;
        }
      }
      else
      {
        std::stringstream ss;
        ss << "No such controller to update: " << state.name;
        getState(result);
        server_->setAborted(result, ss.str());
        return;
      }
    }

    // Update state
    if (state.state == state.STOPPED)
    {
      if (requestStop(state.name) != 0)
      {
        std::stringstream ss;
        ss << "Unable to stop " << state.name;
        getState(result);
        server_->setAborted(result, ss.str());
        return;
      }
    }
    else if (state.state == state.RUNNING)
    {
      if (requestStart(state.name) != 0)
      {
        std::stringstream ss;
        ss << "Unable to start " << state.name;
        getState(result);
        server_->setAborted(result, ss.str());
        return;
      }
    }
    else
    {
      std::stringstream ss;
      ss << "Invalid state for controller " << state.name << ": " << static_cast<int>(state.state);
      getState(result);
      server_->setAborted(result, ss.str());
      return;
    }
  }

  // Send result
  getState(result);
  server_->setSucceeded(result);
}

void ControllerManager::getState(
    robot_controllers_msgs::QueryControllerStatesResult& result)
{
  result.state.clear();
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    robot_controllers_msgs::ControllerState state;
    state.name = (*c)->getController()->getName();
    state.type = (*c)->getController()->getType();
    if ((*c)->isActive())
    {
      state.state = state.RUNNING;
    }
    else
    {
      state.state = state.STOPPED;
    }
    result.state.push_back(state);
  }
}

// NOTE: this function should be called only by one thread
bool ControllerManager::load(const std::string& name)
{
  // Create controller (in a loader)
  ControllerLoaderPtr controller(new ControllerLoader());
  // Push back controller (so that autostart will work)
  controllers_.push_back(controller);
  // Now initialize controller
  if (!controller->init(name, this))
  {
    // Remove if init fails
    controllers_.pop_back();
    return false;
  }
  return true;
}

}  // namespace robot_controllers
