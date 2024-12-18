
#include "auna_gazebo/ground_truth_localization.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Set name for request, if first argument is non-empty.
  if (argc > 1) {
    rclcpp::spin(std::make_shared<GroundTruthLocalization>(argv[1]));
  } else {
    rclcpp::spin(std::make_shared<GroundTruthLocalization>(""));
  }

  rclcpp::shutdown();
  return 0;
}