#include "auna_gazebo/localization_pose_publisher.hpp"

#include <string>
#include <vector>

// Create a publisher, subscriber and prefix. Initialize the transform buffer and listener.
LocalizationPosePublisher::LocalizationPosePublisher(std::string prefix)
: Node("localization_pose_publisher_node"), buffer(this->get_clock()), listener(buffer)
{
  publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose", 2);

  // Declare prefix parameter
  this->declare_parameter("prefix", "");

  // Get parameter from parameter server, default to constructor argument if not set
  std::string param_prefix;
  this->get_parameter("prefix", param_prefix);

  // Use parameter if provided, otherwise use constructor argument
  if (!param_prefix.empty()) {
    prefix = param_prefix;
  }

  // Get namespace from node context as a fallback
  std::string node_namespace = this->get_namespace();
  if (!node_namespace.empty() && node_namespace[0] == '/') {
    node_namespace = node_namespace.substr(1);
  }

  // Select prefix: parameter/argument > namespace > empty
  if (!prefix.empty()) {
    this->prefix = prefix;
    // Add trailing slash if not present
    if (this->prefix.back() != '/') {
      this->prefix += '/';
    }
  } else if (!node_namespace.empty()) {
    this->prefix = node_namespace + "/";
  } else {
    this->prefix = "";
  }

  RCLCPP_INFO(this->get_logger(), "Parameter prefix: '%s'", param_prefix.c_str());
  RCLCPP_INFO(this->get_logger(), "Constructor prefix: '%s'", prefix.c_str());
  RCLCPP_INFO(this->get_logger(), "Node namespace: '%s'", node_namespace.c_str());
  RCLCPP_INFO(this->get_logger(), "Final prefix: '%s'", this->prefix.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing to topic: 'global_pose'");

  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() { timer_callback(); });
}

void LocalizationPosePublisher::timer_callback()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  std::string map_frame = "map";

  RCLCPP_DEBUG(
    this->get_logger(), "Looking up transform: %s -> %s", odom_frame.c_str(), base_frame.c_str());
  try {
    transformStamped = this->buffer.lookupTransform(odom_frame, base_frame, tf2::TimePointZero);
    RCLCPP_DEBUG(this->get_logger(), "Successfully found odom->base_link transform");

    RCLCPP_DEBUG(
      this->get_logger(), "Looking up transform: %s -> %s", map_frame.c_str(), base_frame.c_str());
    transformStamped = this->buffer.lookupTransform(map_frame, base_frame, tf2::TimePointZero);
    RCLCPP_DEBUG(this->get_logger(), "Successfully found map->base_link transform");
  } catch (tf2::TransformException & ex) {
    // RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what()); // Commented out
    // to suppress error when map frame is missing

    // List available frames to help diagnose the issue
    // std::vector<std::string> frames;
    // buffer._getFrameStrings(frames);
    // RCLCPP_INFO(this->get_logger(), "Available frames in TF tree (%zu):", frames.size());
    // for (const auto & frame : frames) {
    //   RCLCPP_INFO(this->get_logger(), " - %s", frame.c_str());
    // }
    return;
  }

  geometry_msgs::msg::PoseStamped localization_pose;
  localization_pose.header.frame_id = "map";
  localization_pose.header.stamp = this->get_clock()->now();
  localization_pose.pose.position.x = transformStamped.transform.translation.x;
  localization_pose.pose.position.y = transformStamped.transform.translation.y;
  localization_pose.pose.position.z = transformStamped.transform.translation.z;
  localization_pose.pose.orientation.x = transformStamped.transform.rotation.x;
  localization_pose.pose.orientation.y = transformStamped.transform.rotation.y;
  localization_pose.pose.orientation.z = transformStamped.transform.rotation.z;
  localization_pose.pose.orientation.w = transformStamped.transform.rotation.w;

  RCLCPP_DEBUG(
    this->get_logger(),
    "Publishing global_pose: position [%.2f, %.2f, %.2f], orientation [%.2f, %.2f, %.2f, %.2f]",
    localization_pose.pose.position.x, localization_pose.pose.position.y,
    localization_pose.pose.position.z, localization_pose.pose.orientation.x,
    localization_pose.pose.orientation.y, localization_pose.pose.orientation.z,
    localization_pose.pose.orientation.w);

  this->publisher->publish(localization_pose);
}
