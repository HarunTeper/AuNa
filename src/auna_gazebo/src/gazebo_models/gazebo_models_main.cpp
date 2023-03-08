#include "auna_gazebo/gazebo_models.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboModels>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}