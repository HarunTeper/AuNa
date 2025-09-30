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

#ifndef AUNA_GROUND_TRUTH__GROUND_TRUTH_POSE_PUBLISHER_HPP_
#define AUNA_GROUND_TRUTH__GROUND_TRUTH_POSE_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class GroundTruthPosePublisher : public rclcpp::Node
{
public:
GroundTruthPosePublisher();

private:
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const;

tf2_ros::Buffer buffer_;
tf2_ros::TransformListener listener_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;

rclcpp::TimerBase::SharedPtr timer_;
void timer_callback();

std::string prefix_;
};

#endif  // AUNA_GROUND_TRUTH__GROUND_TRUTH_POSE_PUBLISHER_HPP_