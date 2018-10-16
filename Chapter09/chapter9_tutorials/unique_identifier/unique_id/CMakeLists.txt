cmake_minimum_required(VERSION 2.8.3)
project(unique_id)

find_package(catkin REQUIRED COMPONENTS roscpp rospy uuid_msgs)
find_package(Boost REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()
catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS uuid_msgs
               DEPENDS Boost)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# unit tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
