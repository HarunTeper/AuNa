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
#include "yaml-cpp/yaml.h"

CaccWaypointPublisher::CaccWaypointPublisher()
: Node("cacc_waypoint_publisher")
{
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

void CaccWaypointPublisher::publish_waypoints()
{
  YAML::Node waypoints;
  try {
    waypoints = YAML::LoadFile(waypoint_file_);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load waypoint file: %s. Error: %s",
      waypoint_file_.c_str(), e.what());
    return;
  }

  auto pose_array_msg = std::make_unique<geometry_msgs::msg::PoseArray>();
  pose_array_msg->header.frame_id = "map";
  pose_array_msg->header.stamp = this->now();

  constexpr bool kApplyRotation = true;
  constexpr double kRotationRadians = -M_PI / 2.0;

  for (const auto & waypoint_node : waypoints) {
    geometry_msgs::msg::Pose pose;

    // Read position
    double x_orig = waypoint_node["position"]["x"].as<double>();
    double y_orig = waypoint_node["position"]["y"].as<double>();
    double z = waypoint_node["position"]["z"].as<double>();

    // Apply rotation if needed
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
    pose.position.z = z;

    // Read orientation
    double qx = waypoint_node["orientation"]["x"].as<double>();
    double qy = waypoint_node["orientation"]["y"].as<double>();
    double qz = waypoint_node["orientation"]["z"].as<double>();
    double qw = waypoint_node["orientation"]["w"].as<double>();

    // Apply rotation to orientation if needed
    if (kApplyRotation) {
      // Convert quaternion to yaw, apply rotation, convert back
      tf2::Quaternion q_orig(qx, qy, qz, qw);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

      double yaw_new = yaw + kRotationRadians;
      if (yaw_new > M_PI) {
        yaw_new -= 2.0 * M_PI;
      }
      if (yaw_new < -M_PI) {
        yaw_new += 2.0 * M_PI;
      }

      tf2::Quaternion q_new;
      q_new.setRPY(0, 0, yaw_new);
      pose.orientation = tf2::toMsg(q_new);
    } else {
      pose.orientation.x = qx;
      pose.orientation.y = qy;
      pose.orientation.z = qz;
      pose.orientation.w = qw;
    }

    pose_array_msg->poses.push_back(pose);
  }

  if (pose_array_msg->poses.empty()) {
    RCLCPP_WARN(
      this->get_logger(), "No waypoints were loaded from %s",
      waypoint_file_.c_str());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Publishing %zu waypoints from %s to %s",
    pose_array_msg->poses.size(), waypoint_file_.c_str(),
    publisher_->get_topic_name());
  publisher_->publish(std::move(pose_array_msg));
}
