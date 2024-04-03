#include "auna_physical/cmd_vel_to_ackermann.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToAckermann>());
    rclcpp::shutdown();
    return 0;
}