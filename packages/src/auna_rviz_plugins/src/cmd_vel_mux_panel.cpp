#include "auna_rviz_plugins/cmd_vel_mux_panel.hpp"

#include <chrono>      // For timer and durations
#include <functional>  // For std::bind

// Qt includes (already in header, but good practice for .cpp)
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>

// ROS includes (already in header)
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>

#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;  // For time units like 100ms

namespace auna_rviz_plugins
{

// Constructor
CmdVelMuxPanel::CmdVelMuxPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  setupUI();
  initializeROS();
}

// Destructor
CmdVelMuxPanel::~CmdVelMuxPanel()
{
  // Qt automatically handles deletion of child widgets when the layout is deleted.
  // Smart pointers for ROS objects handle their own cleanup.
}

// Setup UI elements
void CmdVelMuxPanel::setupUI()
{
  layout_ = new QVBoxLayout(this);  // `this` sets the panel as the layout's parent

  // Namespace Input
  namespace_label_ = new QLabel("Namespace:", this);
  namespace_input_ = new QLineEdit(this);
  namespace_input_->setPlaceholderText("e.g., robot1");
  connect(namespace_input_, &amp; QLineEdit::textChanged, this, &amp;
          CmdVelMuxPanel::onNamespaceChanged);

  // Source Selector
  source_label_ = new QLabel("Command Source:", this);
  source_selector_ = new QComboBox(this);
  source_selector_->addItem("Off", QVariant::fromValue(SelectedSource::OFF));
  source_selector_->addItem("CACC", QVariant::fromValue(SelectedSource::CACC));
  source_selector_->addItem("Teleop", QVariant::fromValue(SelectedSource::TELEOP));
  source_selector_->addItem("Navigation", QVariant::fromValue(SelectedSource::NAV2));
  connect(source_selector_, qOverload<int>(&amp; QComboBox::currentIndexChanged), this, &amp;
          CmdVelMuxPanel::onSourceSelected);

  // Add widgets to layout
  layout_->addWidget(namespace_label_);
  layout_->addWidget(namespace_input_);
  layout_->addWidget(source_label_);
  layout_->addWidget(source_selector_);

  // Set the layout for the panel widget
  setLayout(layout_);
}

// Initialize ROS node, timer
void CmdVelMuxPanel::initializeROS()
{
  // Create a ROS node for the panel.
  // Using rviz_common::Panel::getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()
  // is often preferred to creating a completely separate node, as it integrates better with RViz's
  // lifecycle. However, creating a separate node is simpler initially. Let's stick with that for
  // now.
  node_ = rclcpp::Node::make_shared("cmd_vel_mux_panel_node");

  // Initialize state
  current_source_ = SelectedSource::OFF;
  current_namespace_ = "";  // Start with no namespace

  // Create the publishing timer (e.g., 10 Hz)
  publish_timer_ =
    node_->create_wall_timer(100ms, std::bind(&amp; CmdVelMuxPanel::publishCommand, this));

  // Initial update of subscriptions/publisher based on empty namespace (they won't be valid yet)
  updateSubscriptionsAndPublisher();
}

// Slot for namespace changes
void CmdVelMuxPanel::onNamespaceChanged(const QString & amp; text)
{
  current_namespace_ = text.toStdString();
  RCLCPP_INFO(node_->get_logger(), "Namespace changed to: %s", current_namespace_.c_str());
  updateSubscriptionsAndPublisher();
  // Reset received flags when namespace changes
  cacc_msg_received_ = false;
  teleop_msg_received_ = false;
  nav2_msg_received_ = false;
}

// Slot for source selection changes
void CmdVelMuxPanel::onSourceSelected(int index)
{
  if (index >= 0) {
    QVariant data = source_selector_->itemData(index);
    // Need to properly cast QVariant back to SelectedSource if stored that way
    // For simplicity now, let's use the index directly
    switch (index) {
      case 0:
        current_source_ = SelectedSource::OFF;
        break;
      case 1:
        current_source_ = SelectedSource::CACC;
        break;
      case 2:
        current_source_ = SelectedSource::TELEOP;
        break;
      case 3:
        current_source_ = SelectedSource::NAV2;
        break;
      default:
        current_source_ = SelectedSource::OFF;
        break;
    }
    RCLCPP_INFO(node_->get_logger(), "Source selected: %d", static_cast<int>(current_source_));
  }
}

// Update ROS subscribers and publisher based on the current namespace
void CmdVelMuxPanel::updateSubscriptionsAndPublisher()
{
  // Reset existing pointers to release resources and stop callbacks
  cmd_vel_publisher_.reset();
  cacc_subscriber_.reset();
  teleop_subscriber_.reset();
  nav2_subscriber_.reset();

  if (current_namespace_.empty()) {
    RCLCPP_WARN(
      node_->get_logger(), "Namespace is empty. Publisher and subscribers will not be created.");
    return;
  }

  // Construct topic names with namespace
  std::string cmd_vel_topic = "/" + current_namespace_ + "/cmd_vel";
  std::string cacc_topic = "/" + current_namespace_ + "/cmd_vel_cacc";
  std::string teleop_topic = "/" + current_namespace_ + "/cmd_vel_teleop";
  std::string nav2_topic = "/" + current_namespace_ + "/cmd_vel_nav2";

  // Create publisher
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));                  // Basic QoS
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  // Match RViz default
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  cmd_vel_publisher_ =
    node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, qos_profile);
  RCLCPP_INFO(node_->get_logger(), "Publishing to: %s", cmd_vel_topic.c_str());

  // Create subscribers
  cacc_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cacc_topic, qos_profile,
    std::bind(&amp; CmdVelMuxPanel::caccCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", cacc_topic.c_str());

  teleop_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    teleop_topic, qos_profile,
    std::bind(&amp; CmdVelMuxPanel::teleopCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", teleop_topic.c_str());

  nav2_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    nav2_topic, qos_profile,
    std::bind(&amp; CmdVelMuxPanel::nav2Callback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", nav2_topic.c_str());
}

// Callback for CACC messages
void CmdVelMuxPanel::caccCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cacc_msg_ = *msg;
  cacc_msg_received_ = true;
}

// Callback for Teleop messages
void CmdVelMuxPanel::teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_teleop_msg_ = *msg;
  teleop_msg_received_ = true;
}

// Callback for Nav2 messages
void CmdVelMuxPanel::nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_nav2_msg_ = *msg;
  nav2_msg_received_ = true;
}

// Timer callback to publish the selected command
void CmdVelMuxPanel::publishCommand()
{
  if (!cmd_vel_publisher_ || current_namespace_.empty()) {
    // Don't publish if publisher isn't valid or namespace is not set
    return;
  }

  geometry_msgs::msg::Twist msg_to_publish;

  switch (current_source_) {
    case SelectedSource::CACC:
      if (cacc_msg_received_) {
        msg_to_publish = latest_cacc_msg_;
      } else {
        msg_to_publish = createZeroTwist();  // Publish zero if no msg received yet
      }
      break;
    case SelectedSource::TELEOP:
      if (teleop_msg_received_) {
        msg_to_publish = latest_teleop_msg_;
      } else {
        msg_to_publish = createZeroTwist();
      }
      break;
    case SelectedSource::NAV2:
      if (nav2_msg_received_) {
        msg_to_publish = latest_nav2_msg_;
      } else {
        msg_to_publish = createZeroTwist();
      }
      break;
    case SelectedSource::OFF:
    default:
      msg_to_publish = createZeroTwist();
      break;
  }

  cmd_vel_publisher_->publish(msg_to_publish);
}

// Helper to create a zero Twist message
geometry_msgs::msg::Twist CmdVelMuxPanel::createZeroTwist()
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

// Load configuration - load namespace and selected source
void CmdVelMuxPanel::load(const rviz_common::Config & amp; config)
{
  rviz_common::Panel::load(config);  // Call base class load first

  QString ns;
  int source_index = 0;  // Default to OFF

  if (config.mapGetString("Namespace", &amp; ns)) {
    namespace_input_->setText(ns);  // This will trigger onNamespaceChanged
  }
  if (config.mapGetInt("SourceIndex", &amp; source_index)) {
    if (source_index >= 0 & amp; &amp; source_index < source_selector_->count()) {
      source_selector_->setCurrentIndex(source_index);  // This will trigger onSourceSelected
    }
  }
}

// Save configuration - save namespace and selected source
void CmdVelMuxPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);  // Call base class save first

  config.mapSetValue("Namespace", namespace_input_->text());
  config.mapSetValue("SourceIndex", source_selector_->currentIndex());
}

}  // namespace auna_rviz_plugins

// Export the plugin class. Each plugin must have its own macro statement.
// The ClassLoader automatically creates instances of this object dynamically.
// Usually follows include statement in cpp file
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(auna_rviz_plugins::CmdVelMuxPanel, rviz_common::Panel)
// Note: The GlobalEmergencyStopPanel will need its own export macro later.