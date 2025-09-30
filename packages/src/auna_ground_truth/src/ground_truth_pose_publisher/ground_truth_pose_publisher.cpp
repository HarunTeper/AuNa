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


#include "auna_ground_truth/ground_truth_pose_publisher.hpp"

#include <string>
#include <vector>

// Create a publisher, subscriber and prefix. Initialize the transform buffer and listener.
GroundTruthPosePublisher::GroundTruthPosePublisher()
: Node("localization_pose_publisher_node"), buffer_(this->get_clock()), listener_(buffer_)
{
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gazebo_pose", 2);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() { timer_callback(); });

  RCLCPP_INFO(this->get_logger(), "Publishing to topic: 'gazebo_pose'");
}

void GroundTruthPosePublisher::timer_callback()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  std::string map_frame = "gazebo_world";
  std::string base_frame = "ground_truth_base_link";

  try {
    transformStamped = this->buffer_.lookupTransform(map_frame, base_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    // Suppress error output
    return;
  }

  geometry_msgs::msg::PoseStamped localization_pose;
  localization_pose.header.frame_id = map_frame;
  localization_pose.header.stamp = this->get_clock()->now();
  localization_pose.pose.position.x = transformStamped.transform.translation.x;
  localization_pose.pose.position.y = transformStamped.transform.translation.y;
  localization_pose.pose.position.z = transformStamped.transform.translation.z;
  localization_pose.pose.orientation.x = transformStamped.transform.rotation.x;
  localization_pose.pose.orientation.y = transformStamped.transform.rotation.y;
  localization_pose.pose.orientation.z = transformStamped.transform.rotation.z;
  localization_pose.pose.orientation.w = transformStamped.transform.rotation.w;

  this->publisher_->publish(localization_pose);
}
