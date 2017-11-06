cmake_minimum_required(VERSION 2.8.3)
project(pointgrey_generate_tests)

set(LADYBUG_SDK "" CACHE PATH "SDK ROOT DIR" )

include_directories (${LADYBUG_SDK}/include)
add_executable(generate main.cpp)
target_link_libraries(generate 
	${LADYBUG_SDK}/lib/ladybug.lib
	${LADYBUG_SDK}/lib/freeglut.lib
	${LADYBUG_SDK}/lib/LadybugGUI.lib)

