#include "auna_gazebo/localization_pose_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationPosePublisher>());
  rclcpp::shutdown();
  return 0;
}