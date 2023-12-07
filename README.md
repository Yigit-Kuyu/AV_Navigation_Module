# Aim
This is the modular version of [AV_Navigation](https://github.com/Yigit-Kuyu/AV_Navigation) package. Similar to it, this package takes a map as pgm file and implements A* algorithm to find a root with two options (with/without direction). In differences, this module publishes the custom message that includes x and y coordinates of the pgm file on "AV_Navigation" topic. The file, subs_coord.cpp, is designed to listen to these messages on the same topic, with ROS Noetic, Ubuntu 20.04, C++ 17.

# Implementation for astar-gridmap-2d_ros1 (main module)
  ```
/astar_subscribe_ws$ mkdir src
/astar_subscribe_ws$ catkin_make
/astar_subscribe_ws/src$ catkin_init_workspace
/astar_subscribe_ws/src$ catkin_create_pkg subs_coord std_msgs roscpp geometry_msgs
/astar_subscribe_ws$ catkin_make
/astar_subscribe_ws$ rosrun subs_coord run_subs 

  ```
# Implementation for astar_subscribe_ws (subscriber)
  ```
/ros1_cpp_ws$ mkdir src
/ros1_cpp_ws$ catkin_make
/ros1_cpp_ws/src$ catkin_init_workspace
/ros1_cpp_ws$ catkin_create_pkg ros_package std_msgs roscpp
/ros1_cpp_ws$ catkin_make
/ros1_cpp_ws$ rosrun ros_navigation run_pkg 


  ```

![AV_Module](https://github.com/Yigit-Kuyu/AV_Navigation_Module/blob/main/Module.PNG)

