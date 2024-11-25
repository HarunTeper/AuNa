#include "auna_rviz_plugins/control_panel.hpp"

namespace auna_rviz_plugins
{

ControlPanel::ControlPanel(QWidget* parent) : rviz_common::Panel(parent)
{
  // Create the three buttons
  QPushButton* emergencyStopButton = new QPushButton("Emergency Stop");
  QPushButton* changeVelocityButton = new QPushButton("Change Velocity");
  QPushButton* unusedButton = new QPushButton("Unused Button");

  // Connect button signals to their respective slots
  connect(emergencyStopButton, &QPushButton::clicked, this, &ControlPanel::onEmergencyStopClicked);
  connect(changeVelocityButton, &QPushButton::clicked, this, &ControlPanel::onChangeVelocityClicked);

  // Create a vertical layout and add the buttons
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(emergencyStopButton);
  layout->addWidget(changeVelocityButton);
  layout->addWidget(unusedButton);

  // Set the layout to the panel
  setLayout(layout);

  // Create a separate ROS2 node
  node_ = std::make_shared<rclcpp::Node>("my_plugin_panel_node");
}

void ControlPanel::onEmergencyStopClicked()
{
  // Create the service client
  auto client = node_->create_client<std_srvs::srv::Empty>("/emergency_stop_enable");

  // Wait for the client to be ready
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Service not available.");
    return;
  }

  // Create the service request
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  // Send the service request
  auto result = client->async_send_request(request);
  rclcpp::spin_until_future_complete(node_, result);

  if (result.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed.");
  }
}

void ControlPanel::onChangeVelocityClicked()
{
  // Get the desired velocity from the user input (assuming QLineEdit named 'velocity_input_')
  float desired_velocity = velocity_input_->text().toFloat();

  // Create a ROS2 parameter client
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node_);

  // Change the 'desired_velocity' parameter
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("desired_velocity", desired_velocity));
  auto results = param_client->set_parameters(parameters);

  for (const auto& result : results)
  {
    if (!result.successful)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
    }
  }
}
  
} // namespace auna_rviz_plugins

#include <pluginlib/class_list_macros.hpp> 
PLUGINLIB_EXPORT_CLASS(auna_rviz_plugins::ControlPanel, rviz_common::Panel)