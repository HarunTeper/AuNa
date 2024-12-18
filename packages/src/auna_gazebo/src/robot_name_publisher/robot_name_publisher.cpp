#include "auna_gazebo/robot_name_publisher.hpp"

RobotNamePublisher::RobotNamePublisher() : Node("robot_name_publisher_node")
{
  subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
    "/model_states", 2,
    [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) { model_state_callback(msg); });
  publisher_ = this->create_publisher<auna_msgs::msg::StringArray>("/robot_names", 2);
}

void RobotNamePublisher::model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  auto message = auna_msgs::msg::StringArray();
  for (std::string model_name : msg->name) {
    if (model_name.find("robot") != std::string::npos) {
      message.strings.push_back(model_name);
    }
  }
  publisher_->publish(message);
}
