#include "auna_ground_truth/ground_truth_pose_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthPosePublisher>());
  rclcpp::shutdown();
  return 0;
}