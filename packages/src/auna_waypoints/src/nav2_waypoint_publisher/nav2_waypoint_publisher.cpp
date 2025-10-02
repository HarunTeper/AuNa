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

#include "auna_waypoints/nav2_waypoint_publisher.hpp"

WaypointPublisher::WaypointPublisher()
    : Node("nav2_waypoint_publisher"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                   [this]() { timer_callback(); });

  this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, "navigate_through_poses");

  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter<std::string>("namespace", namespace_);
  this->declare_parameter<std::string>("waypoint_file", "");
  this->get_parameter<std::string>("waypoint_file", waypoint_file_);

  try {
    YAML::Node waypoints_yaml = YAML::LoadFile(waypoint_file_);

    if (!waypoints_yaml.IsSequence()) {
      RCLCPP_ERROR(this->get_logger(),
                   "YAML file should contain a sequence of waypoints");
      return;
    }

    for (const auto& waypoint_node : waypoints_yaml) {
      if (!waypoint_node["position"]) {
        RCLCPP_WARN(this->get_logger(),
                    "Waypoint missing position field, skipping");
        continue;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";

      // Parse position
      const auto& position = waypoint_node["position"];
      pose.pose.position.x = position["x"].as<double>();
      pose.pose.position.y = position["y"].as<double>();
      pose.pose.position.z =
          position["z"].as<double>(0.0);  // Default to 0.0 if not specified

      // Parse orientation
      if (waypoint_node["orientation"]) {
        const auto& orientation = waypoint_node["orientation"];

        // Check if all required orientation components are present
        if (!orientation["x"] || !orientation["y"] || !orientation["z"] ||
            !orientation["w"]) {
          RCLCPP_WARN(this->get_logger(),
                      "Waypoint has incomplete orientation (missing x, y, z, "
                      "or w), using default");
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;
        } else {
          pose.pose.orientation.x = orientation["x"].as<double>();
          pose.pose.orientation.y = orientation["y"].as<double>();
          pose.pose.orientation.z = orientation["z"].as<double>();
          pose.pose.orientation.w = orientation["w"].as<double>();
        }
      } else {
        // Default orientation (facing positive x)
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
      }

      poses_.push_back(pose);
    }

  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse YAML waypoint file: %s",
                 e.what());
    return;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading waypoint file: %s",
                 e.what());
    return;
  }
  // Validate that we have waypoints loaded
  if (poses_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No waypoints loaded from file: %s",
                 waypoint_file_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from file: %s",
              poses_.size(), waypoint_file_.c_str());

  // to start from zero at first iteration
  current_pose_index_ = -number_of_waypoints_;
}

void WaypointPublisher::timer_callback() {
  // Check if the action client is available and ready
  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN(this->get_logger(),
                "Action server not available, skipping waypoint publishing");
    return;
  }

  if (remaining_number_of_poses_ < number_of_waypoints_ * 0.5) {
    publish_waypoints();
  }
}

void WaypointPublisher::publish_waypoints() {
  RCLCPP_INFO(this->get_logger(), "Publishing waypoints");

  // Safety check: ensure we have waypoints loaded
  if (poses_.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Cannot publish waypoints: no waypoints loaded!");
    return;
  }

  current_pose_index_ = (current_pose_index_ + number_of_waypoints_ -
                         remaining_number_of_poses_) %
                        static_cast<int>(poses_.size());

  auto goal_msg = NavigateThroughPoses::Goal();
  for (int i = current_pose_index_;
       i < current_pose_index_ + number_of_waypoints_; i++) {
    goal_msg.poses.push_back(poses_[i % poses_.size()]);
  }

  auto send_goal_options =
      rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
      &WaypointPublisher::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&WaypointPublisher::feedback_callback, this,
                std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(
      &WaypointPublisher::result_callback, this, std::placeholders::_1);

  RCLCPP_INFO(this->get_logger(), "Sending goal");
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void WaypointPublisher::goal_response_callback(
    GoalHandleNavigateThroughPoses::SharedPtr goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received goal response callback");
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
    remaining_number_of_poses_ = number_of_waypoints_;
  }
}

void WaypointPublisher::feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
  // RCLCPP_INFO(this->get_logger(), "Received feedback: %i",
  // feedback->number_of_poses_remaining);
  remaining_number_of_poses_ = feedback->number_of_poses_remaining;
}

void WaypointPublisher::result_callback(
    const GoalHandleNavigateThroughPoses::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      publish_waypoints();
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      publish_waypoints();
      return;
  }
  RCLCPP_INFO(this->get_logger(), "Navigation finished");
}
