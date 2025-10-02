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

#include "auna_physical/waypoint_publisher.hpp"

WaypointPublisher::WaypointPublisher()
: Node("waypoint_publisher"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]() {timer_callback();});

  this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
    this, "navigate_through_poses");

  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter<std::string>("namespace", namespace_);
  this->declare_parameter<std::string>("waypoint_file", "");
  this->get_parameter<std::string>("waypoint_file", waypoint_file_);

  std::ifstream file(waypoint_file_, std::ifstream::in);
  std::string line;
  bool first_line = true;
  if (file.is_open()) {
    while (std::getline(file, line)) {
      // Remove carriage return characters (Windows line endings)
      line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

      // Handle BOM (Byte Order Mark) and other non-printable characters at the
      // beginning of file
      if (first_line) {
        // Remove UTF-8 BOM (EF BB BF) and other potential non-printable
        // characters
        while (!line.empty() &&
          (static_cast<unsigned char>(line[0]) > 127 ||
          line[0] == '\xEF' || line[0] == '\xBB' || line[0] == '\xBF' ||
          (line[0] >= 0 && line[0] < 32 && line[0] != '\t')))
        {
          line.erase(0, 1);
        }
        first_line = false;
      }

      // Skip empty lines or lines with only whitespace
      if (line.empty() ||
        line.find_first_not_of(" \t\n\r") == std::string::npos)
      {
        continue;
      }

      std::istringstream iss(line);
      std::string x, y;
      std::getline(iss, x, ',');
      std::getline(iss, y, ',');

      // Trim whitespace from x and y coordinates
      x.erase(0, x.find_first_not_of(" \t"));
      x.erase(x.find_last_not_of(" \t") + 1);
      y.erase(0, y.find_first_not_of(" \t"));
      y.erase(y.find_last_not_of(" \t") + 1);

      // Skip if either coordinate is empty after trimming
      if (x.empty() || y.empty()) {
        continue;
      }

      try {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = std::stof(x);
        pose.pose.position.y = std::stof(y);
        pose.pose.position.z = 0;

        double heading = 0;
        pose.pose.orientation =
          tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));

        poses_.push_back(pose);
      } catch (const std::invalid_argument & e) {
        RCLCPP_WARN(
          this->get_logger(), "Invalid waypoint data in line: '%s'",
          line.c_str());
        continue;
      } catch (const std::out_of_range & e) {
        RCLCPP_WARN(
          this->get_logger(),
          "Waypoint coordinate out of range in line: '%s'",
          line.c_str());
        continue;
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unable to open file");
    return;
  }
  file.close();
  for (uint i = 0; i < poses_.size(); i++) {
    if (i == 0) {
      double heading = atan2(
        poses_[i + 1].pose.position.y -
        poses_[poses_.size() - 1].pose.position.y,
        poses_[i + 1].pose.position.x -
        poses_[poses_.size() - 1].pose.position.x);
      poses_[i].pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
    } else if (i == poses_.size() - 1) {
      double heading =
        atan2(
        poses_[0].pose.position.y - poses_[i - 1].pose.position.y,
        poses_[0].pose.position.x - poses_[i - 1].pose.position.x);
      poses_[i].pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
    } else {
      double heading =
        atan2(
        poses_[i + 1].pose.position.y - poses_[i - 1].pose.position.y,
        poses_[i + 1].pose.position.x - poses_[i - 1].pose.position.x);
      poses_[i].pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
    }
  }

  // to start from zero at first iteration
  current_pose_index_ = -number_of_waypoints_;
}

void WaypointPublisher::timer_callback()
{
  // Check if the action client is available and ready
  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN(
      this->get_logger(),
      "Action server not available, skipping waypoint publishing");
    return;
  }

  if (remaining_number_of_poses_ < number_of_waypoints_ * 0.5) {
    publish_waypoints();
  }
}

void WaypointPublisher::publish_waypoints()
{
  RCLCPP_INFO(this->get_logger(), "Publishing waypoints");

  current_pose_index_ = (current_pose_index_ + number_of_waypoints_ -
    remaining_number_of_poses_) %
    poses_.size();

  auto goal_msg = NavigateThroughPoses::Goal();
  for (int i = current_pose_index_;
    i < current_pose_index_ + number_of_waypoints_; i++)
  {
    goal_msg.poses.push_back(poses_[i % poses_.size()]);
  }

  auto send_goal_options =
    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
    &WaypointPublisher::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &WaypointPublisher::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(
    &WaypointPublisher::result_callback, this, std::placeholders::_1);

  RCLCPP_INFO(this->get_logger(), "Sending goal");
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void WaypointPublisher::goal_response_callback(
  GoalHandleNavigateThroughPoses::SharedPtr goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal response callback");
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Goal accepted by server, waiting for result");
    remaining_number_of_poses_ = number_of_waypoints_;
  }
}

void WaypointPublisher::feedback_callback(
  GoalHandleNavigateThroughPoses::SharedPtr,
  const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
  // RCLCPP_INFO(this->get_logger(), "Received feedback: %i",
  // feedback->number_of_poses_remaining);
  remaining_number_of_poses_ = feedback->number_of_poses_remaining;
}

void WaypointPublisher::result_callback(
  const GoalHandleNavigateThroughPoses::WrappedResult & result)
{
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
