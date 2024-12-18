#include "auna_gazebo/robot_name_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNamePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}