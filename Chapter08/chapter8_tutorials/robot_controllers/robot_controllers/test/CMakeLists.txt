catkin_add_gtest(trajectory_tests trajectory_tests.cpp)
target_link_libraries(trajectory_tests
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

