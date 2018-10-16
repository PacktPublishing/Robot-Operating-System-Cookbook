cmake_minimum_required(VERSION 2.8.3)
project(robot_controllers_interface)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(Boost REQUIRED system)
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    pluginlib
    roscpp
    robot_controllers_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    pluginlib
    roscpp
    robot_controllers_msgs
  DEPENDS
    Boost
  LIBRARIES
    robot_controllers_interface
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(robot_controllers_interface
  src/controller_loader.cpp
  src/controller_manager.cpp
)
target_link_libraries(robot_controllers_interface
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(robot_controllers_interface
  robot_controllers_msgs_generate_messages_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS robot_controllers_interface LIBRARY
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  PROGRAMS
    scripts/get_controller_state.py
    scripts/start_controller.py
    scripts/stop_controller.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
