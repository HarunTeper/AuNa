#include "auna_comm/cam_communication.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamCommunication>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}