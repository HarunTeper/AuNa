#include "auna_mqtt/mqtt_waypoint_receiver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MQTTWaypointReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}