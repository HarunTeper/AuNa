#include "auna_rviz_plugins/global_emergency_stop_panel.hpp"

#include <chrono>
#include <functional>
#include <sstream>  // For splitting string
#include <string>
#include <vector>

// Qt includes
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtGui/QColor>    // For button color
#include <QtGui/QPalette>  // For button color
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>

#include <geometry_msgs/msg/twist.hpp>

namespace auna_rviz_plugins
{

// Constructor
GlobalEmergencyStopPanel::GlobalEmergencyStopPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  setupUI();
  initializeROS();
}

// Destructor
GlobalEmergencyStopPanel::~GlobalEmergencyStopPanel()
{
  // Qt handles UI cleanup
  // ROS smart pointers handle their cleanup
}

// Setup UI elements
void GlobalEmergencyStopPanel::setupUI()
{
  layout_ = new QVBoxLayout(this);

  // Namespaces Input
  namespaces_label_ = new QLabel("Target Namespaces (comma-separated):", this);
  namespaces_input_ = new QLineEdit(this);
  namespaces_input_->setPlaceholderText("e.g., robot1,robot2");
  connect(
    namespaces_input_, &QLineEdit::textChanged, this,
    &GlobalEmergencyStopPanel::onNamespacesChanged);

  // Emergency Stop Button
  emergency_stop_button_ = new QPushButton("GLOBAL EMERGENCY STOP", this);
  // Make button visually distinct (e.g., red background) - requires QPalette/Stylesheet
  QPalette pal = emergency_stop_button_->palette();
  pal.setColor(QPalette::Button, QColor(Qt::red));
  pal.setColor(QPalette::ButtonText, QColor(Qt::white));
  emergency_stop_button_->setAutoFillBackground(true);
  emergency_stop_button_->setPalette(pal);
  emergency_stop_button_->update();  // Apply palette
  connect(
    emergency_stop_button_, &QPushButton::clicked, this,
    &GlobalEmergencyStopPanel::onGlobalEmergencyStopClicked);

  // Add widgets to layout
  layout_->addWidget(namespaces_label_);
  layout_->addWidget(namespaces_input_);
  layout_->addWidget(emergency_stop_button_);

  setLayout(layout_);
}

// Initialize ROS node
void GlobalEmergencyStopPanel::initializeROS()
{
  // Create a dedicated node for this panel
  node_ = rclcpp::Node::make_shared("global_emergency_stop_panel_node");
  updatePublishers();  // Initialize publishers based on potentially loaded text
}

// Slot for namespace input changes
void GlobalEmergencyStopPanel::onNamespacesChanged(const QString & text)
{
  target_namespaces_ = text.split(',', Qt::SkipEmptyParts);  // Split by comma, remove empty entries
  for (QString & ns : target_namespaces_) {
    ns = ns.trimmed();  // Remove leading/trailing whitespace
  }
  target_namespaces_.removeAll(QString(""));  // Ensure no empty strings remain

  RCLCPP_INFO(node_->get_logger(), "Target namespaces updated.");
  updatePublishers();
}

// Slot for emergency stop button click
void GlobalEmergencyStopPanel::onGlobalEmergencyStopClicked()
{
  RCLCPP_WARN(node_->get_logger(), "GLOBAL EMERGENCY STOP ACTIVATED!");
  geometry_msgs::msg::Twist zero_twist = createZeroTwist();

  if (publisher_map_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No target namespaces set, cannot send stop command.");
    return;
  }

  for (auto const & [ns, pub] : publisher_map_) {
    if (pub) {
      RCLCPP_INFO(node_->get_logger(), "Sending stop command to namespace: %s", ns.c_str());
      pub->publish(zero_twist);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Publisher for namespace %s is invalid!", ns.c_str());
    }
  }
  // Optional: Add visual feedback like flashing the button
}

// Update the map of publishers based on target_namespaces_
void GlobalEmergencyStopPanel::updatePublishers()
{
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> new_publisher_map;
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  for (const QString & q_ns : target_namespaces_) {
    std::string ns = q_ns.toStdString();
    if (ns.empty()) continue;  // Skip empty namespaces just in case

    // Check if publisher already exists
    auto it = publisher_map_.find(ns);
    if (it != publisher_map_.end()) {
      // Reuse existing publisher
      new_publisher_map[ns] = it->second;
      publisher_map_.erase(it);  // Remove from old map to track remaining unused ones
    } else {
      // Create new publisher
      std::string topic_name = "/" + ns + "/cmd_vel";
      try {
        new_publisher_map[ns] =
          node_->create_publisher<geometry_msgs::msg::Twist>(topic_name, qos_profile);
        RCLCPP_INFO(node_->get_logger(), "Created publisher for: %s", topic_name.c_str());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to create publisher for %s: %s", topic_name.c_str(),
          e.what());
        new_publisher_map[ns] = nullptr;  // Store nullptr to indicate failure
      }
    }
  }

  // Any publishers left in the old publisher_map_ are no longer needed
  if (!publisher_map_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Removing %zu unused publishers.", publisher_map_.size());
  }
  publisher_map_.clear();  // Clear the old map (smart pointers handle resource release)

  // Swap the new map into place
  publisher_map_.swap(new_publisher_map);
}

// Helper to create a zero Twist message
geometry_msgs::msg::Twist GlobalEmergencyStopPanel::createZeroTwist()
{
  geometry_msgs::msg::Twist zero_twist;
  zero_twist.linear.x = 0.0;
  zero_twist.linear.y = 0.0;
  zero_twist.linear.z = 0.0;
  zero_twist.angular.x = 0.0;
  zero_twist.angular.y = 0.0;
  zero_twist.angular.z = 0.0;
  return zero_twist;
}

// Load configuration
void GlobalEmergencyStopPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns_list;
  if (config.mapGetString("TargetNamespaces", &ns_list)) {
    namespaces_input_->setText(ns_list);  // This triggers onNamespacesChanged -> updatePublishers
  }
}

// Save configuration
void GlobalEmergencyStopPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("TargetNamespaces", namespaces_input_->text());
}

}  // namespace auna_rviz_plugins

// Export the plugin class
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(auna_rviz_plugins::GlobalEmergencyStopPanel, rviz_common::Panel)