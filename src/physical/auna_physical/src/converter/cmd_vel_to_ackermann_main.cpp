#include "auna_physical/cmd_vel_to_ackermann.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Set prefix for topic and frame_id, if first argument is non-empty.
    rclcpp::spin(std::make_shared<CmdVelToAckermann>());
    rclcpp::shutdown();
    return 0;
}