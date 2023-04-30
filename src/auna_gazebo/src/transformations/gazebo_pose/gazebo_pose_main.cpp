#include "auna_gazebo/gazebo_pose.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    

    // Set name for request, if first argument is non-empty.
    if(argc > 1)
    {
      rclcpp::spin(std::make_shared<GazeboPose>(argv[1]));
    }
    else
    {
      rclcpp::spin(std::make_shared<GazeboPose>(""));
    }

    rclcpp::shutdown();
    return 0;
}