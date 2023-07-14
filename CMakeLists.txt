cmake_minimum_required(VERSION 3.0.2)
project(catkin_ic)

set(CMAKE_CXX_STANDARD 17)


find_package(gazebo REQUIRED)
find_package(catkin REQUIRED)
find_package(roscpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)


include_directories(${catkin_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${CMAKE-EXTRAS_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})


link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(teste ~/ros/src/catkin_ic/scripts/teste.cc)
target_link_libraries(teste ${GAZEBO_LIBRARIES}  ${roscpp_LIBRARIES})


catkin_package()

