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

#ifndef AUNA_PHYSICAL__NAV2_WAYPOINT_PUBLISHER_HPP_
#define AUNA_PHYSICAL__NAV2_WAYPOINT_PUBLISHER_HPP_

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateThroughPoses =
  rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class WaypointPublisher : public rclcpp::Node
{
public:
  WaypointPublisher();

private:
  // create a namespace variable
  std::string namespace_;

  // create a buffer and listener for tf2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // create a timer and its callback
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  // publish waypoints
  void publish_waypoints();

  // action client and callbacks
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
    client_ptr_;
  void goal_response_callback(
    GoalHandleNavigateThroughPoses::SharedPtr goal_handle);
  void feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
  void result_callback(
    const GoalHandleNavigateThroughPoses::WrappedResult & result);

  // waypoint data
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::string waypoint_file_;
  int current_pose_index_ = 0;
  int remaining_number_of_poses_ = 0;
  bool remaining_decreased_ = false;
  int number_of_waypoints_ = 20;
};

#endif  // AUNA_PHYSICAL__NAV2_WAYPOINT_PUBLISHER_HPP_
