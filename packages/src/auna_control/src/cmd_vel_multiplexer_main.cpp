#include "auna_control/cmd_vel_multiplexer_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auna_control::CmdVelMultiplexerNode>());
  rclcpp::shutdown();
  return 0;
}