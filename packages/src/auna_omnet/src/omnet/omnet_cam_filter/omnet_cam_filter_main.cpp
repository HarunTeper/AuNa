#include "auna_omnet/omnet_cam_filter.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Set prefix for topic and frame_id, if first argument is non-empty.
    if(argc > 1)
    {
      rclcpp::spin(std::make_shared<OmnetCamFilter>(std::stoi(argv[1])));
    }
    else
    {
      rclcpp::spin(std::make_shared<OmnetCamFilter>(0));
    }
    rclcpp::shutdown();
    return 0;
}