cmake_minimum_required(VERSION 2.8.3)
project(pointgrey)


set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
find_package(OpenCV REQUIRED)

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
	${OpenCV_INCLUDE_DIRS}
)

add_executable(lady_bug_16bit
	ladybug_16bitTest.cpp
)



target_link_libraries(lady_bug_16bit
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
   flycapture
   ladybug
   xerces-c
)


