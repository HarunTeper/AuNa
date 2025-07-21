#include "auna_tf/localization_pose_publisher.hpp"

#include <string>
#include <vector>

// Create a publisher, subscriber and prefix. Initialize the transform buffer and listener.
LocalizationPosePublisher::LocalizationPosePublisher()
: Node("localization_pose_publisher_node"), buffer_(this->get_clock()), listener_(buffer_)
{
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose", 2);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() { timer_callback(); });

  RCLCPP_INFO(this->get_logger(), "Publishing to topic: 'global_pose'");
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
    transformStamped = this->buffer_.lookupTransform(odom_frame, base_frame, tf2::TimePointZero);
    RCLCPP_DEBUG(this->get_logger(), "Successfully found odom->base_link transform");

    RCLCPP_DEBUG(
      this->get_logger(), "Looking up transform: %s -> %s", map_frame.c_str(), base_frame.c_str());
    transformStamped = this->buffer_.lookupTransform(map_frame, base_frame, tf2::TimePointZero);
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

  this->publisher_->publish(localization_pose);
}
