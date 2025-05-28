
#include "auna_gazebo/ground_truth_localization.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<GroundTruthLocalization>());

  rclcpp::shutdown();
  return 0;
}