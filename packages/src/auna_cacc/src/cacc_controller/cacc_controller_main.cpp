#include "auna_cacc/cacc_controller.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CaccController>());
    rclcpp::shutdown();
    return 0;
}