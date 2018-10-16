cmake_minimum_required(VERSION 2.8.3)
project(robot_controllers_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    message_generation
    std_msgs
)

add_message_files(
  FILES
    ControllerState.msg
    DiffDriveLimiterParams.msg
)

add_action_files(
  FILES
    QueryControllerStates.action
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
    std_msgs
)
