#include "auna_pure_pursuit/auna_pure_pursuit_controller.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AuNaPurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}