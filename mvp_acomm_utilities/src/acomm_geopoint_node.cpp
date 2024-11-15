#include "ros/ros.h"

#include "acomm_geopoint.hpp"


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "acomm_geopoint");

  AcommGeoPoint d;  

  ros::spin();

  ros::shutdown();
  return 0;
}