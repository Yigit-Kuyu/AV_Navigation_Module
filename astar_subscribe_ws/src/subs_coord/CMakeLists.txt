cmake_minimum_required(VERSION 3.0.2)
project(subs_coord)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
)


add_message_files(
  FILES
  navigation_yck.msg )


generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(run_subs
src/subs_coord.cpp
)

target_link_libraries(run_subs ${catkin_LIBRARIES})

add_dependencies(run_subs subs_coord_generate_messages_cpp)
# add_dependencies(source_file_name package_name_generate_messages_cpp)  # To recognize msg

