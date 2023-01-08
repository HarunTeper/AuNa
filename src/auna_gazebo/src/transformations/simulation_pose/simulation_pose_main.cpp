#include "auna_gazebo/simulation_pose.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    

    // Set name for request, if first argument is non-empty.
    if(argc > 1)
    {
      rclcpp::spin(std::make_shared<SimulationPose>(argv[1]));
    }
    else
    {
      rclcpp::spin(std::make_shared<SimulationPose>(""));
    }

    rclcpp::shutdown();
    return 0;
}