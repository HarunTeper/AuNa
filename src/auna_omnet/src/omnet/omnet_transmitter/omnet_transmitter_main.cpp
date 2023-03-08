#include "auna_omnet/omnet_transmitter.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // Set prefix for topic and frame_id, if first argument is non-empty.
    if(argc > 1)
    {
      rclcpp::spin(std::make_shared<OmnetTransmitter>(argv[1]));
    }
    else
    {
      rclcpp::spin(std::make_shared<OmnetTransmitter>(""));
    }

    rclcpp::shutdown();
    return 0;
}