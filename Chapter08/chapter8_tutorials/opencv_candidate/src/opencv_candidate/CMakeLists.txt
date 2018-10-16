add_library(opencv_candidate datamatrix.cpp
                             feature2d.cpp
)

target_link_libraries(opencv_candidate
  ${OpenCV_LIBS}
  )
set_target_properties(opencv_candidate PROPERTIES COMPILE_FLAGS "-fopenmp" LINK_FLAGS "-fopenmp")

#set_target_properties(opencv_candidate PROPERTIES COMPILE_FLAGS "-msse4.2")

install(TARGETS opencv_candidate
    DESTINATION lib
    )
