find_package(Boost REQUIRED thread system)

include_directories(SYSTEM /opt/DepthSenseSDK/include
                           ${OpenCV_INCLUDE_DIRS}
                           ${BOOST_INCLUDE_DIRS}
)

find_library(DEPTH_SENSE_LIBRARIES DepthSense /opt/DepthSenseSDK/lib)
find_library(TURBO_JPEG_LIBRARIES turbojpeg /opt/DepthSenseSDK/lib)


add_library(opencv_creative SHARED creative.cpp)

target_link_libraries(opencv_creative ${Boost_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ${DEPTH_SENSE_LIBRARIES}
                                      ${TURBO_JPEG_LIBRARIES}
)

install(TARGETS opencv_creative
        DESTINATION lib
)

add_executable(main main.cpp)
target_link_libraries(main opencv_creative
)
