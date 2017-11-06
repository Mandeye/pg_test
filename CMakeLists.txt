cmake_minimum_required(VERSION 2.8.3)
project(pointgrey)

add_subdirectory (gtest)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} include )

###########
## Build ##
###########

if(EXISTS "/usr/include/flycapture")
include_directories(
	/usr/include/flycapture
	${OpenCV_INCLUDE_DIRS}
)
add_executable(grasshopper3 nodes/grasshopper3/grasshopper3.cpp)

target_link_libraries(grasshopper3
   ${catkin_LIBRARIES} ${OpenCV_LIBS} flycapture
)
else()
message("'flycapture2' is not installed. 'grasshopper3' will not be built.")
endif()




#add_executable(ladybug_pps_service
#	nodes/ladybug/ladybug_pps_service.cpp
#)
#add_dependencies(ladybug_pps_service pointgrey_generate_messages_cpp)

#target_link_libraries(ladybug_pps_service
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   flycapture
#   ladybug
#   xerces-c
#)

include_directories(
	/usr/include/ladybug
	nodes/ladybug
	ladybugStitchFrom3DMesh
	${OpenCV_INCLUDE_DIRS}
)

add_executable(lady_bug_16bit
	ladybug_16bitTest.cpp
)

add_executable(lady_bug_8bit
	ladybug_8bitTest.cpp
)
add_executable(lady_bug_8bit_rect
	ladybug_8bitTest_rect.cpp
)
add_executable(ladybugStitchFrom3DMesh
 ladybugStitchFrom3DMesh/ladybugStitchFrom3DMesh.cpp
 ladybugStitchFrom3DMesh/pgrpnmio.cpp
)
add_executable(ladybugStitchFrom3DMeshCamera
 ladybugStitchFrom3DMeshCamera.cpp
 ladybugStitchFrom3DMesh/pgrpnmio.cpp
)


target_link_libraries(lady_bug_16bit
   ${OpenCV_LIBS}
   flycapture
   ladybug
   xerces-c
)
target_link_libraries(lady_bug_8bit
   ${OpenCV_LIBS}
   flycapture
   ladybug
   xerces-c
)
target_link_libraries(lady_bug_8bit_rect
   ${OpenCV_LIBS}
   flycapture
   ladybug
   xerces-c
)
target_link_libraries(ladybugStitchFrom3DMesh 
    ${OpenCV_LIBS} 
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY}
)

target_link_libraries(ladybugStitchFrom3DMeshCamera 
    ${OpenCV_LIBS} 
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY}
    flycapture
    ladybug
    xerces-c
)

file(GLOB_RECURSE test_sources  test/*.cpp)
add_executable(test_${PROJECT_NAME} ${test_sources})
target_link_libraries(test_${PROJECT_NAME} ${GTEST_LIBRARIES} pthread gtest
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)
