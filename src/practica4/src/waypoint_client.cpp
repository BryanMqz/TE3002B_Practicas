#include "ros/ros.h"
#include "practica4/waypointinfo.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_client");
  if (argc != 3)
  {
    ROS_INFO("usage: waypoint_client info");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<practica4::waypointinfo>("waypoint");
  practica4::waypointinfo srv;
  
  if (client.call(srv))
  {
    ROS_INFO("Waypoint Distance: %ld", (double)srv.response.distance);
    ROS_INFO("Completed Waypoints: %ld", (long int)srv.response.completeWP);
  }
  else
  {
    ROS_ERROR("Failed to call service waypoint");
    return 1;
  }

  return 0;
}