// Copyright 2025 Harun Teper
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include "auna_tf/global_tf.hpp"

#include "rclcpp/rclcpp.hpp"  // Ensure rclcpp is included for logging macros

// Create a service callback to the Gazebo models to get the current robot names.
GlobalTF::GlobalTF() : Node("global_tf_node"), tf_broadcaster_(this), static_tf_broadcaster_(this)
{
  service_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(100), [this]() { service_timer_callback(); });
  modelClient_ = this->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
}

// Callback for the timer. Creates a service call to get the Gazebo model names.
void GlobalTF::service_timer_callback()
{
  auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
  while (!modelClient_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  auto result = modelClient_->async_send_request(
    request, std::bind(&GlobalTF::model_srv_callback, this, std::placeholders::_1));
}

// Service callback of the Gazebo models.
void GlobalTF::model_srv_callback(
  const rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedFuture future)
{
  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get model list from Gazebo service.");
    return;
  }
  for (std::string model_name : result.get()->model_names) {
    // Only check for names that include robot
    if (model_name.find("robot") != std::string::npos) {
      // For each model name, check if already added. If not, add subscribers for local tf topics.
      if (
        std::find(robot_models_.begin(), robot_models_.end(), model_name) == robot_models_.end()) {
        robot_models_.push_back(model_name);
        RCLCPP_INFO(
          this->get_logger(), "GLOBAL_TF: Creating TF subscriptions for robot: %s",
          model_name.c_str());

        // Use capturing lambda to pass the robot name to the callback
        tf_subscribers_.push_back(this->create_subscription<tf2_msgs::msg::TFMessage>(
          "/" + model_name + "/tf", rclcpp::QoS(10),  // Increased QoS depth
          [this, robot_name = model_name](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            tf_callback(msg, robot_name, false);  // Dynamic transform
          }));
        // Define QoS profile for static transforms (compatible with transient_local)
        auto static_qos =
          rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();  // Increased QoS depth
        tf_subscribers_.push_back(this->create_subscription<tf2_msgs::msg::TFMessage>(
          "/" + model_name + "/tf_static", static_qos,
          [this, robot_name = model_name](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            tf_callback(msg, robot_name, true);  // Static transform
          }));
      }
    }
  }
}

// Callback for local tf topics. Publishes the transformations to the global tf topic.
void GlobalTF::tf_callback(
  const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string & robot_name, bool is_static)
{
  for (const geometry_msgs::msg::TransformStamped & message : msg->transforms) {
    // if (!is_static) {
    //   RCLCPP_INFO(
    //     this->get_logger(),
    //     "GLOBAL_TF DYNAMIC (%s): Received original: %s -> %s (Timestamp: %d.%09d)",
    //     robot_name.c_str(), message.header.frame_id.c_str(), message.child_frame_id.c_str(),
    //     message.header.stamp.sec, message.header.stamp.nanosec);
    // } else {
    //   RCLCPP_INFO(  // Also log static for completeness, but less verbosely
    //     this->get_logger(), "GLOBAL_TF STATIC (%s): Received original: %s -> %s",
    //     robot_name.c_str(), message.header.frame_id.c_str(), message.child_frame_id.c_str());
    // }

    geometry_msgs::msg::TransformStamped modified = message;
    std::string original_header = message.header.frame_id;
    std::string original_child = message.child_frame_id;

    // Case 1: map -> odom transform (Typically dynamic, but could be static if odom is fixed
    // relative to map temporarily)
    if (original_header == "map" && original_child == "odom") {
      modified.header.frame_id = "map";
      modified.child_frame_id = robot_name + "/odom";
    }
    else if (original_header == "gazebo_world" && original_child == "odom") {
      modified.header.frame_id = "gazebo_world";
      modified.child_frame_id = robot_name + "/odom";
    }
    else if (original_header == "gazebo_world" && original_child == "map") {
      modified.header.frame_id = "gazebo_world";
      modified.child_frame_id = "map";
    }
    else if (original_header == "gazebo_world" && original_child == "ground_truth_base_link") {
      modified.header.frame_id = "gazebo_world";
      modified.child_frame_id = robot_name + "/ground_truth_base_link";
    }
    // Case 3: odom -> base_link transform (Typically dynamic)
    else if (
      original_header == "odom" &&
      (original_child == "base_link" || original_child == "base_footprint")) {
      modified.header.frame_id = robot_name + "/odom";
      modified.child_frame_id = robot_name + "/" + original_child;
    }
    // Case 4: Other transforms (robot-internal, could be static or dynamic)
    else {
      // Prefix header if not already prefixed AND it's not a global frame like 'map'
      if (modified.header.frame_id.rfind(robot_name + "/", 0) != 0 && original_header != "map") {
        modified.header.frame_id = robot_name + "/" + modified.header.frame_id;
      }
      // Prefix child if not already prefixed
      if (modified.child_frame_id.rfind(robot_name + "/", 0) != 0) {
        modified.child_frame_id = robot_name + "/" + modified.child_frame_id;
      }
    }

    // Ensure the timestamp from the original message is preserved
    modified.header.stamp = message.header.stamp;

    // if (!is_static) {
    //   RCLCPP_INFO(
    //     this->get_logger(),
    //     "GLOBAL_TF DYNAMIC (%s): Attempting to publish modified: %s -> %s (Timestamp: %d.%09d)",
    //     robot_name.c_str(), modified.header.frame_id.c_str(), modified.child_frame_id.c_str(),
    //     modified.header.stamp.sec, modified.header.stamp.nanosec);
    // } else {
    //   RCLCPP_INFO(
    //     this->get_logger(), "GLOBAL_TF STATIC (%s): Attempting to publish modified: %s -> %s",
    //     robot_name.c_str(), modified.header.frame_id.c_str(), modified.child_frame_id.c_str());
    // }

    if (is_static) {
      static_tf_broadcaster_.sendTransform(modified);
    } else {
      tf_broadcaster_.sendTransform(modified);
    }
  }
}
