#include "auna_omnet/omnet_receiver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmnetReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}