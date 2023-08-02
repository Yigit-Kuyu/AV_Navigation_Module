#include "subs_coord/navigation_yck.h" 
#include "geometry_msgs/Point.h"
#include <vector>

#include "ros/ros.h"


void subs(const subs_coord::navigation_yck::ConstPtr& msg) {
    int i=msg->another_field;
    ROS_INFO(" Listening, Counter %d", msg->another_field);
    ROS_INFO(" x=%.2f, y=%.2f", msg->points[i].x, msg->points[i].y);
}


int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("AV_navigation", 1, subs);

  ros::spin();

}