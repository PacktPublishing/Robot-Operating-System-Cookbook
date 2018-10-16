cmake_minimum_required(VERSION 2.8.3)
project(fh_desc)
find_package(catkin REQUIRED COMPONENTS urdf xacro rospy)

catkin_python_setup()

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS urdf xacro rospy
    INCLUDE_DIRS
    LIBRARIES
)

install(DIRECTORY meshes/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
