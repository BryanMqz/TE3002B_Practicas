#include "ros/ros.h"
#include "practica4/waypointinfo.h"
#include <std_srvs/EmptyResponse.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

float distance_;
int completeWP_;

bool info(practica4::waypointinfo::Request  &req,
         practica4::waypointinfo::Response &res)
{
  ROS_INFO("sending back response: distance=[%lf], completeWP=[%ld]", (double)res.distance, (long int)res.completeWP);
  return true;
}

bool cb(const std_msgs::EmptyConstPtr)
{
    distance_ = 0;
    completeWP_ = 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_server");
  ros::NodeHandle nh;
  ros::Subscriber wp_info=nh.subscribe("/waypoint_Info", 10, cb);
  ros::ServiceServer service = nh.advertiseService("waypoint", info);
  ROS_INFO("Waypoint Info Ready.");
  ros::spin();

  return 0;
}