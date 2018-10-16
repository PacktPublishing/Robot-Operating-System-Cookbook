# merge some RGBD functions into a module
#set(the_description "a set of RGBD functions")
#ocv_define_module(rgbd)

find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  return()
endif()

add_library(opencv_rgbd src/normal.cpp
                        src/depth_to_3d.cpp
                        src/depth_cleaner.cpp
                        src/odometry.cpp
                        src/plane.cpp
                        src/rgbd_init.cpp
                        src/utils.cpp
)

target_link_libraries(opencv_rgbd ${OpenCV_LIBS}
                                  ${OpenCV_LIBRARIES}
)

install(TARGETS opencv_rgbd
        DESTINATION lib
)

install(DIRECTORY include/opencv2/rgbd
        DESTINATION include/opencv2
        COMPONENT main
)

# Add sample
add_executable(odometry_evaluation samples/odometry_evaluation.cpp)
target_link_libraries(odometry_evaluation ${OpenCV_LIBRARIES} opencv_rgbd)

# Add some tests
return()
add_executable(rgbd_tests test/test_main.cpp
                          test/test_normal.cpp
                          test/test_odometry.cpp
                          test/test_precomp.cpp
                          test/test_utils.cpp
)

find_library(OPENCV_TS_LIBRARY opencv_ts ${OpenCV_LIB_DIR})
target_link_libraries(rgbd_tests ${OpenCV_LIBRARIES}
                                 opencv_rgbd
                                 ${OPENCV_TS_LIBRARY}
)

add_test(rgbd_tests rgbd_tests)
