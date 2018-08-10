cmake_minimum_required(VERSION 2.8.3)
project(serial)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        DEPENDS rt pthread
    )

set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)

## Add serial library
add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)



add_executable(serial_ros examples/serial_ros.cpp)
target_link_libraries(serial_ros ${catkin_LIBRARIES})
add_dependencies(serial_ros ${PROJECT_NAME})
target_link_libraries(serial_ros ${PROJECT_NAME})

add_executable(serial_example examples/serial_example.cc)
add_dependencies(serial_example ${PROJECT_NAME})
target_link_libraries(serial_example ${PROJECT_NAME})

add_executable(serial_xx examples/serial_xx.cpp)
add_dependencies(serial_xx ${PROJECT_NAME})
target_link_libraries(serial_xx ${PROJECT_NAME})

add_executable(serial_ros_1 examples/serial_ros_1.cpp)
target_link_libraries(serial_ros_1 ${catkin_LIBRARIES})
##add_dependencies(serial_ros package_name_generate_messages_cpp)
add_dependencies(serial_ros_1 ${PROJECT_NAME})
target_link_libraries(serial_ros_1 ${PROJECT_NAME})

add_executable(serial_ros_xx examples/serial_ros_xx.cpp)
target_link_libraries(serial_ros_xx ${catkin_LIBRARIES})
##add_dependencies(serial_ros package_name_generate_messages_cpp)
add_dependencies(serial_ros_xx ${PROJECT_NAME})
target_link_libraries(serial_ros_xx ${PROJECT_NAME})


add_executable(talker examples/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
##add_dependencies(serial_ros package_name_generate_messages_cpp)
add_dependencies(talker ${PROJECT_NAME})
target_link_libraries(talker ${PROJECT_NAME})

add_executable(talker_xx examples/talker_xx.cpp)
target_link_libraries(talker_xx ${catkin_LIBRARIES})
add_dependencies(talker_xx ${PROJECT_NAME})
target_link_libraries(talker_xx ${PROJECT_NAME})

add_executable(serial_ros_xx1 examples/serial_ros_xx1.cpp)
target_link_libraries(serial_ros_xx1 ${catkin_LIBRARIES})
add_dependencies(serial_ros_xx1 ${PROJECT_NAME})
target_link_libraries(serial_ros_xx1 ${PROJECT_NAME})




## Include headers
include_directories(include)

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/serial/serial.h include/serial/v8stdint.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/serial)

## Tests
if(CATKIN_ENABLE_TESTING)
    add_subdirectory(tests)
endif()


