# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(geodesy)
find_package(catkin REQUIRED
             COMPONENTS
             angles
             geographic_msgs
             geometry_msgs
             sensor_msgs
             tf
             unique_id
             uuid_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_python_setup()

# what other packages will need to use this one
catkin_package(CATKIN_DEPENDS
               geographic_msgs
               geometry_msgs
               sensor_msgs
               tf
               unique_id
               uuid_msgs
               INCLUDE_DIRS include
               LIBRARIES geoconv)

# build C++ coordinate conversion library
add_library(geoconv src/conv/utm_conversions.cpp)
target_link_libraries(geoconv ${catkin_LIBRARIES})
add_dependencies(geoconv ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS geoconv
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# unit tests
if (CATKIN_ENABLE_TESTING)

  catkin_add_gtest(test_wgs84 tests/test_wgs84.cpp)
  target_link_libraries(test_wgs84 ${catkin_LIBRARIES})
  add_dependencies(test_wgs84 ${catkin_EXPORTED_TARGETS})
  
  catkin_add_gtest(test_utm tests/test_utm.cpp)
  target_link_libraries(test_utm geoconv ${catkin_LIBRARIES})
  add_dependencies(test_utm ${catkin_EXPORTED_TARGETS})
  
  catkin_add_nosetests(tests/test_bounding_box.py)
  catkin_add_nosetests(tests/test_props.py)
  catkin_add_nosetests(tests/test_utm.py)
  catkin_add_nosetests(tests/test_wu_point.py)

endif (CATKIN_ENABLE_TESTING)
