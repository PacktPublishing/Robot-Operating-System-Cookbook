cmake_minimum_required(VERSION 2.8.3)
project(rotors_simulator_demos)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs mav_msgs roscpp sensor_msgs sensor_fusion_comm)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    mav_msgs
    roscpp
    sensor_msgs
    sensor_fusion_comm
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(hovering_example_msf src/hovering_example_msf.cpp)
target_link_libraries(hovering_example_msf ${catkin_LIBRARIES})
add_dependencies(hovering_example_msf ${catkin_EXPORTED_TARGETS})
