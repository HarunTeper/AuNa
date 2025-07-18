#include "auna_ground_truth/ground_truth_cam.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<GroundTruthCam>());

  rclcpp::shutdown();
  return 0;
}