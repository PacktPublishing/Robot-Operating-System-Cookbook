cmake_minimum_required(VERSION 2.8.3)
project(robot_controllers)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wno-unused-parameter")
endif()

find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED system)
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_msgs
    angles
    control_msgs
    geometry_msgs
    kdl_parser
    nav_msgs
    pluginlib
    robot_controllers_interface
    roscpp
    sensor_msgs
    tf
    tf_conversions
    trajectory_msgs
    urdf
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    control_msgs
    geometry_msgs
    kdl_parser
    nav_msgs
    pluginlib
    robot_controllers_interface
    roscpp
    sensor_msgs
    tf
    tf_conversions
    trajectory_msgs
    urdf
  DEPENDS
    Boost
    orocos_kdl
  LIBRARIES
    robot_controllers
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# this is a hack, will eventually be unneeded once orocos-kdl is fixed
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(robot_controllers
  src/cartesian_pose.cpp
  src/cartesian_twist.cpp
  src/cartesian_wrench.cpp
  src/diff_drive_base.cpp
  src/follow_joint_trajectory.cpp
  src/gravity_compensation.cpp
  src/parallel_gripper.cpp
  src/pid.cpp
  src/point_head.cpp
  src/scaled_mimic.cpp
)
target_link_libraries(robot_controllers
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

install(DIRECTORY include/ DESTINATION include)

install(FILES robot_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS robot_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
