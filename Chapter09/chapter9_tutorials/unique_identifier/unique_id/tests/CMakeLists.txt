### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# C++ gtests
catkin_add_gtest(test_${PROJECT_NAME}
                 test_${PROJECT_NAME}.cpp
                 second_test_${PROJECT_NAME}.cpp)
add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_${PROJECT_NAME}
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})

# Python nose tests
catkin_add_nosetests(test_${PROJECT_NAME}.py)
