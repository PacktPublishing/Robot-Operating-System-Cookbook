find_package(libg2o QUIET)

if (NOT ${libg2o_FOUND})
return()
endif()

find_package(OpenCV REQUIRED)

# Model capture stuff
find_package(PCL REQUIRED COMPONENTS io filters visualization features segmentation surface)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS}
                           /usr/include/suitesparse
)

# Find Eigen3
FIND_PACKAGE(Eigen)
find_package(libg2o)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

# add reconst3d library
file(GLOB reconst3d_sources "*.cpp")

add_library(opencv_candidate_reconst3d ${reconst3d_sources})
target_link_libraries(opencv_candidate_reconst3d ${catkin_LIBRARIES}
                                                 ${OpenCV_LIBRARIES}
                                                 opencv_rgbd
                                                 opencv_candidate
                                                 ${libg2o_LIBRARIES}
                                                 ${PCL_LIBRARIES}
                                                 cholmod
                                                 cxsparse
)
set_target_properties(opencv_candidate_reconst3d PROPERTIES COMPILE_FLAGS "-fopenmp" LINK_FLAGS "-fopenmp")

# add model capture samples
add_executable(circular_model_capture samples/circular_model_capture.cpp)
target_link_libraries(circular_model_capture ${OpenCV_LIBRARIES}
                                    opencv_rgbd
                                    opencv_candidate_reconst3d
)
set_target_properties(circular_model_capture PROPERTIES COMPILE_FLAGS "-fopenmp" LINK_FLAGS "-fopenmp")

add_executable(arbitrary_model_capture samples/arbitrary_model_capture.cpp)
target_link_libraries(arbitrary_model_capture ${OpenCV_LIBRARIES}
                                    opencv_rgbd
                                    opencv_candidate_reconst3d
)
set_target_properties(arbitrary_model_capture PROPERTIES COMPILE_FLAGS "-fopenmp" LINK_FLAGS "-fopenmp")

add_executable(cylinder_evaluation samples/cylinder_evaluation.cpp)
target_link_libraries(cylinder_evaluation ${OpenCV_LIBRARIES}
                                    opencv_rgbd
                                    opencv_candidate_reconst3d
)
set_target_properties(cylinder_evaluation PROPERTIES COMPILE_FLAGS "-fopenmp" LINK_FLAGS "-fopenmp")
