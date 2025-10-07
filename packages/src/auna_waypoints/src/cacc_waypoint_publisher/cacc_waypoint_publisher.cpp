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

#include "auna_waypoints/cacc_waypoint_publisher.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

#include "rclcpp/qos.hpp"

CaccWaypointPublisher::CaccWaypointPublisher()
    : Node("cacc_waypoint_publisher") {
  this->declare_parameter<std::string>("waypoint_file", "");
  this->get_parameter("waypoint_file", waypoint_file_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  std::string topic_name = "/cacc/waypoints";
  publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>(topic_name, qos);

  // Publish waypoints once during initialization
  // The transient_local QoS policy ensures late subscribers receive the message
  publish_waypoints();
}

void CaccWaypointPublisher::publish_waypoints() {
  std::ifstream file(waypoint_file_);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s",
                 waypoint_file_.c_str());
    return;
  }

  auto pose_array_msg = std::make_unique<geometry_msgs::msg::PoseArray>();
  pose_array_msg->header.frame_id = "map";
  pose_array_msg->header.stamp = this->now();

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;
    std::vector<double> values;
    while (std::getline(ss, value, ',')) {
      try {
        values.push_back(std::stod(value));
      } catch (const std::invalid_argument& ia) {
        continue;
      }
    }

    if (values.size() >= 2) {
      geometry_msgs::msg::Pose pose;

      double x_orig = values[0];
      double y_orig = values[1];

      constexpr bool kApplyRotation = true;
      constexpr double kRotationRadians = -M_PI / 2.0;
      double xr = x_orig;
      double yr = y_orig;
      if (kApplyRotation) {
        xr = x_orig * std::cos(kRotationRadians) -
             y_orig * std::sin(kRotationRadians);
        yr = x_orig * std::sin(kRotationRadians) +
             y_orig * std::cos(kRotationRadians);
      }
      pose.position.x = xr;
      pose.position.y = yr;
      pose.position.z = 0.0;

      if (values.size() >= 3) {
        double yaw_orig = values[2];
        double yaw_new = yaw_orig + (kApplyRotation ? kRotationRadians : 0.0);
        if (yaw_new > M_PI) {
          yaw_new -= 2.0 * M_PI;
        }
        if (yaw_new < -M_PI) {
          yaw_new += 2.0 * M_PI;
        }

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_new);
        pose.orientation = tf2::toMsg(q);
      } else {
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        pose.orientation = tf2::toMsg(q);
      }
      pose_array_msg->poses.push_back(pose);
    }
  }

  if (pose_array_msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No waypoints were loaded from %s",
                waypoint_file_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Publishing %zu waypoints from %s to %s",
              pose_array_msg->poses.size(), waypoint_file_.c_str(),
              publisher_->get_topic_name());
  publisher_->publish(std::move(pose_array_msg));
}
