#include "auna_comm/cam_receiver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
