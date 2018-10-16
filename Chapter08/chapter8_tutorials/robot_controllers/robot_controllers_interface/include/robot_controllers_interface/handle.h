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
 *     * Neither the name of the FFetch Robotics Inc. nor the names of its
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

#ifndef ROBOT_CONTROLLERS_INTERFACE_HANDLE_H
#define ROBOT_CONTROLLERS_INTERFACE_HANDLE_H

#include <string>
#include <boost/shared_ptr.hpp>

namespace robot_controllers
{

/**
 * @brief Base class for a handle.
 */
class Handle
{
public:
  Handle()
  {
  }

  /** @brief Ensure proper cleanup with virtual destructor. */
  virtual ~Handle()
  {
  }

  /** @brief Get the name of this joint. */
  virtual std::string getName() = 0;

private:
  // No copy
  Handle(const Handle&);
  Handle& operator=(const Handle&);
};

typedef boost::shared_ptr<Handle> HandlePtr;

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_INTERFACE_HANDLE_H
