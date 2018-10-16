cmake_minimum_required(VERSION 2.8)
project(opencv_candidate)

find_package(catkin REQUIRED)
find_package(libg2o QUIET)
find_package(OpenCV)
find_library(DEPTH_SENSE_LIBRARIES DepthSense /opt/DepthSenseSDK/lib)

if (${DEPTH_SENSE_LIBRARIES} STREQUAL DEPTH_SENSE_LIBRARIES-NOTFOUND)
  set(OPENCV_CREATIVE "")
else()
  set(OPENCV_CREATIVE opencv_creative)
endif()

if (libg2o_FOUND)
  set(EXTRA_DEPENDS libg2o)
  set(EXTRA_LIBRARIES opencv_candidate_reconst3d)
endif()

if(OpenCV_VERSION VERSION_LESS "3.0")
set(RGB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/rgbd/include)
endif()

catkin_package(DEPENDS OpenCV ${EXTRA_DEPENDS}
               INCLUDE_DIRS include ${RGB_INCLUDE_DIRS}
               LIBRARIES opencv_candidate
                         ${OPENCV_CREATIVE}
                         opencv_rgbd
                         ${opencv_candidate_reconst3d}
)

find_package(OpenCV REQUIRED core imgproc highgui calib3d)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV headers: ${OpenCV_INCLUDE_DIRS}")
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(include)

# install the include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/rgbd/include)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/opencv_candidate
        DESTINATION include
        COMPONENT main
)

add_subdirectory(src)
