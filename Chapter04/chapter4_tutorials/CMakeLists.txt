cmake_minimum_required(VERSION 2.8.3)
project(chapter4_tutorials)

set(ROS_BUILD_TYPE Debug)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      message_generation
      std_msgs
      geometry_msgs
      sensor_msgs
      visualization_msgs
      dynamic_reconfigure
      diagnostic_updater
      cv_bridge
      image_transport
      pcl_conversions
      interactive_markers)

find_package(OpenCV)
find_package(PCL REQUIRED)

add_service_files(FILES SetSpeed.srv)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(cfg/DynamicParam.cfg)

catkin_package(
    CATKIN_DEPENDS
      roscpp
      message_runtime
      std_msgs
      geometry_msgs
      sensor_msgs
      visualization_msgs
      dynamic_reconfigure
      diagnostic_updater
      cv_bridge
      image_transport
      pcl_conversions
      interactive_markers)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(program1 src/program1.cpp)
target_link_libraries(program1 ${catkin_LIBRARIES})

add_executable(program1_dump src/program1_dump.cpp)
target_link_libraries(program1_dump ${catkin_LIBRARIES})

add_executable(program1_mem src/program1_mem.cpp)
target_link_libraries(program1_mem ${catkin_LIBRARIES})

add_executable(program2 src/program2.cpp)
target_link_libraries(program2 ${catkin_LIBRARIES})

add_executable(program3 src/program3.cpp)
target_link_libraries(program3 ${catkin_LIBRARIES})

add_executable(program4 src/program4.cpp)
add_dependencies(program4 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(program4 ${catkin_LIBRARIES})

add_executable(program5 src/program5.cpp)
add_dependencies(program5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(program5 ${catkin_LIBRARIES})

add_executable(program6 src/program6.cpp)
add_dependencies(program6 ${PROJECT_NAME}_gencfg)
target_link_libraries(program6 ${catkin_LIBRARIES})

add_executable(program7 src/program7.cpp)
target_link_libraries(program7 ${catkin_LIBRARIES})

add_executable(program8 src/program8.cpp)
target_link_libraries(program8 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(program9 src/program9.cpp)
target_link_libraries(program9 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(program10 src/program10.cpp)
target_link_libraries(program10 ${catkin_LIBRARIES})

install(TARGETS
    program1 program1_dump program1_mem program2 program3 program4 program5 program6 program7 program8
    program9 program10
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir bag config launch output)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
