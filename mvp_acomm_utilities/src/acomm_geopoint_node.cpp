#include "rclcpp/rclcpp.hpp"

#include "acomm_geopoint.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<AcommGeoPoint> node = std::make_shared<AcommGeoPoint>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}