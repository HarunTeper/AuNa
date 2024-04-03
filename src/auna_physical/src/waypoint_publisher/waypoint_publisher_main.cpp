#include "auna_physical/waypoint_publisher.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}