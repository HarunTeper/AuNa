#include "auna_physical/nav2_waypoint_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2WaypointPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}