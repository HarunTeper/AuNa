#include "auna_physical/vesc_start.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VescStart>());
    rclcpp::shutdown();
    return 0;
}