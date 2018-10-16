#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import sys
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from robot_controllers_msgs.msg import QueryControllerStatesAction, \
                                       QueryControllerStatesGoal, \
                                       ControllerState

ACTION_NAME = "/query_controller_states"

if __name__ == "__main__":
    rospy.init_node("get_robot_controllers_state")

    rospy.loginfo("Connecting to %s..." % ACTION_NAME)
    client = actionlib.SimpleActionClient(ACTION_NAME, QueryControllerStatesAction)
    client.wait_for_server()

    rospy.loginfo("Requesting state of controllers...")

    goal = QueryControllerStatesGoal()
    client.send_goal(goal)
    client.wait_for_result()
    if client.get_state() == GoalStatus.SUCCEEDED:
        result = client.get_result()
        for state in result.state:
            if state.state == state.RUNNING:
                print("%s[%s]: RUNNING" % (state.name, state.type))
            elif state.state == state.STOPPED:
                print("%s[%s]: STOPPED" % (state.name, state.type))
            elif state.state == state.ERROR:
                print("%s[%s]: ERROR!!" % (state.name, state.type))
    elif client.get_state() == GoalStatus.ABORTED:
        rospy.logerr(client.get_goal_status_text())
