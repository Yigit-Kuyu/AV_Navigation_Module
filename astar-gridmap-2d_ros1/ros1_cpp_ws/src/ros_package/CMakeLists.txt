cmake_minimum_required(VERSION 3.0.2)
project(ros_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
  
)

find_package(OpenCV REQUIRED)


add_message_files(
  FILES
  navigation_yck.msg

)

generate_messages(
  DEPENDENCIES
  geometry_msgs  
  #   std_msgs
)


catkin_package(CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(run_pkg 
src/benchmark_ros.cpp
src/util.h
src/benchmark.h
src/AStar2.h
src/AStar2.cpp

)
target_link_libraries(run_pkg ${catkin_LIBRARIES} ${OpenCV_LIBS})


