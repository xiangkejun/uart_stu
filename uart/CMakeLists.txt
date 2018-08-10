cmake_minimum_required(VERSION 2.8.3)
project(test)


if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)

include_directories(src) 
FILE(GLOB_RECURSE sourcefiles "src/*.c")
add_executable( uart ${sourcefiles} )

#add_executable(uart src/main.c)





