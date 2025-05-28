
#include "auna_gazebo/ground_truth_transform.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<GroundTruthTransform>());

  rclcpp::shutdown();
  return 0;
}