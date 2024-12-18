#include "rclcpp/rclcpp.hpp"

#include "auna_msgs/msg/string_array.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

class RobotNamePublisher : public rclcpp::Node
{
public:
  RobotNamePublisher();

private:
  void model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
  rclcpp::Publisher<auna_msgs::msg::StringArray>::SharedPtr publisher_;
};
