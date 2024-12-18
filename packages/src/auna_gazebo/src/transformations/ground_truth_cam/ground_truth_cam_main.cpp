#include "auna_gazebo/ground_truth_cam.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Set name for request, if first argument is non-empty.
  if (argc > 1) {
    rclcpp::spin(std::make_shared<GroundTruthCam>(argv[1]));
  } else {
    rclcpp::spin(std::make_shared<GroundTruthCam>(""));
  }

  rclcpp::shutdown();
  return 0;
}