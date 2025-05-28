#include "auna_gazebo/global_tf.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalTF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}