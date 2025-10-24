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
  tf_listener_(tf_buffer_)
{
  // Publisher for the next waypoint being navigated to
  next_waypoint_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "next_waypoint",
    10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]() {timer_callback();});

  this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
    this, "navigate_through_poses");

  // Initialize waypoint array publisher with transient_local QoS
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
  qos_profile.transient_local();
  this->waypoint_array_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(
    "nav2_waypoints",
    qos_profile);

  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter<std::string>("namespace", namespace_);
  this->declare_parameter<std::string>("waypoint_file", "");
  this->get_parameter<std::string>("waypoint_file", waypoint_file_);

  try {
    YAML::Node waypoints_yaml = YAML::LoadFile(waypoint_file_);

    if (!waypoints_yaml.IsSequence()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "YAML file should contain a sequence of waypoints");
      return;
    }

    for (const auto & waypoint_node : waypoints_yaml) {
      if (!waypoint_node["position"]) {
        RCLCPP_WARN(
          this->get_logger(),
          "Waypoint missing position field, skipping");
        continue;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";

      // Parse position
      const auto & position = waypoint_node["position"];
      pose.pose.position.x = position["x"].as<double>();
      pose.pose.position.y = position["y"].as<double>();
      pose.pose.position.z =
        position["z"].as<double>(0.0);    // Default to 0.0 if not specified

      // Parse orientation
      if (waypoint_node["orientation"]) {
        const auto & orientation = waypoint_node["orientation"];

        // Check if all required orientation components are present
        if (!orientation["x"] || !orientation["y"] || !orientation["z"] ||
          !orientation["w"])
        {
          RCLCPP_WARN(
            this->get_logger(),
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
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to parse YAML waypoint file: %s",
      e.what());
    return;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Error loading waypoint file: %s",
      e.what());
    return;
  }
  // Validate that we have waypoints loaded
  if (poses_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "No waypoints loaded from file: %s",
      waypoint_file_.c_str());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Loaded %zu waypoints from file: %s",
    poses_.size(), waypoint_file_.c_str());

  // Publish the complete waypoint array
  publish_waypoint_array();

  // Create a timer to periodically republish the waypoint array for
  // visualization
  waypoint_array_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), [this]() {publish_waypoint_array();});

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

  RCLCPP_DEBUG(
    this->get_logger(),
    "Timer callback: remaining_number_of_poses_=%d, threshold=%d",
    remaining_number_of_poses_, waypoint_refresh_threshold_);

  // Trigger new waypoint publishing when threshold or fewer waypoints remain
  if (remaining_number_of_poses_ <= waypoint_refresh_threshold_) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Timer triggering waypoint publishing (remaining: %d <= %d)",
      remaining_number_of_poses_, waypoint_refresh_threshold_);
    publish_waypoints();
  }
}

void WaypointPublisher::publish_waypoints()
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Publishing waypoints. Current pose index: '%d, "
    "number_of_waypoints: %d, remaining_number_of_poses: %d'",
    current_pose_index_, number_of_waypoints_,
    remaining_number_of_poses_);

  // Safety check: ensure we have waypoints loaded
  if (poses_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Cannot publish waypoints: no waypoints loaded!");
    return;
  }

  // Get robot's current pose
  geometry_msgs::msg::PoseStamped robot_pose = get_robot_pose();

  RCLCPP_DEBUG(
    this->get_logger(), "Robot pose: (x=%.2f, y=%.2f, theta=%.2f)",
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf2::getYaw(robot_pose.pose.orientation));

  // Find nearest waypoint with matching orientation
  int nearest_index = find_nearest_waypoint_with_orientation(robot_pose);
  if (nearest_index >= 0) {
    current_pose_index_ = nearest_index;
    RCLCPP_INFO(
      this->get_logger(),
      "Selected nearest waypoint with matching orientation: %d "
      "(x=%.2f, y=%.2f)",
      current_pose_index_,
      poses_[current_pose_index_].pose.position.x,
      poses_[current_pose_index_].pose.position.y);
  } else {
    // Fallback to original logic if no suitable waypoint found
    current_pose_index_ = (current_pose_index_ + number_of_waypoints_ -
      remaining_number_of_poses_) %
      static_cast<int>(poses_.size());
    RCLCPP_WARN(
      this->get_logger(),
      "No waypoint with matching orientation found, using fallback "
      "logic. Fallback index: %d (x=%.2f, y=%.2f)",
      current_pose_index_,
      poses_[current_pose_index_].pose.position.x,
      poses_[current_pose_index_].pose.position.y);
  }

  // Publish the next waypoint being navigated to
  if (!poses_.empty() && current_pose_index_ >= 0 &&
    current_pose_index_ < static_cast<int>(poses_.size()))
  {
    next_waypoint_publisher_->publish(poses_[current_pose_index_]);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Published next waypoint: (x=%.2f, y=%.2f, theta=%.2f)",
      poses_[current_pose_index_].pose.position.x,
      poses_[current_pose_index_].pose.position.y,
      tf2::getYaw(poses_[current_pose_index_].pose.orientation));
  }

  auto goal_msg = NavigateThroughPoses::Goal();

  RCLCPP_INFO(this->get_logger(), "Waypoints to be sent in this goal:");
  for (int i = current_pose_index_;
    i < current_pose_index_ + number_of_waypoints_; i++)
  {
    const auto & pose = poses_[i % poses_.size()];
    RCLCPP_INFO(
      this->get_logger(), "  [%d] (x=%.2f, y=%.2f, theta=%.2f)",
      static_cast<int>(i % poses_.size()), pose.pose.position.x,
      pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
    goal_msg.poses.push_back(pose);
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

  RCLCPP_INFO(
    this->get_logger(),
    "Sending goal to nav2 NavigateThroughPoses action server");
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void WaypointPublisher::goal_response_callback(
  GoalHandleNavigateThroughPoses::SharedPtr goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal response callback");
  if (!goal_handle) {
    RCLCPP_ERROR(
      this->get_logger(), "Goal was rejected by server (index: %d)",
      current_pose_index_);
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Goal accepted by server, waiting for result (index: %d)",
      current_pose_index_);
    remaining_number_of_poses_ = number_of_waypoints_;
  }
}

void WaypointPublisher::feedback_callback(
  GoalHandleNavigateThroughPoses::SharedPtr,
  const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
  RCLCPP_DEBUG(
    this->get_logger(), "Received feedback: %d poses remaining",
    feedback->number_of_poses_remaining);
  remaining_number_of_poses_ = feedback->number_of_poses_remaining;
}

void WaypointPublisher::result_callback(
  const GoalHandleNavigateThroughPoses::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        this->get_logger(), "Goal succeeded for waypoint index: %d",
        current_pose_index_);
      // Continue to next set of waypoints
      remaining_number_of_poses_ = 0;  // Trigger timer to publish new waypoints
      RCLCPP_INFO(
        this->get_logger(),
        "Navigation will continue to next waypoints on next timer tick");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(
        this->get_logger(),
        "Goal was aborted for waypoint index: %d (x=%.2f, y=%.2f)",
        current_pose_index_,
        poses_[current_pose_index_].pose.position.x,
        poses_[current_pose_index_].pose.position.y);
      // publish_waypoints();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(
        this->get_logger(),
        "Goal was canceled for waypoint index: %d (x=%.2f, y=%.2f)",
        current_pose_index_,
        poses_[current_pose_index_].pose.position.x,
        poses_[current_pose_index_].pose.position.y);
      // publish_waypoints();
      return;
    default:
      RCLCPP_ERROR(
        this->get_logger(),
        "Unknown result code (%d) for waypoint index: %d (x=%.2f, y=%.2f)",
        static_cast<int>(result.code), current_pose_index_,
        poses_[current_pose_index_].pose.position.x,
        poses_[current_pose_index_].pose.position.y);
      // publish_waypoints();
      return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Navigation finished for waypoint index: %d",
    current_pose_index_);
}

void WaypointPublisher::publish_waypoint_array()
{
  geometry_msgs::msg::PoseArray waypoint_array;
  waypoint_array.header.frame_id = "map";
  waypoint_array.header.stamp = this->get_clock()->now();

  // Get robot's current pose to find nearest waypoint
  geometry_msgs::msg::PoseStamped robot_pose = get_robot_pose();

  // Find nearest waypoint with matching orientation
  int nearest_index = find_nearest_waypoint_with_orientation(robot_pose);

  // If no waypoint found with matching orientation, find closest by distance
  // only
  if (nearest_index < 0 && !poses_.empty()) {
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < poses_.size(); ++i) {
      double dx = poses_[i].pose.position.x - robot_pose.pose.position.x;
      double dy = poses_[i].pose.position.y - robot_pose.pose.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < min_distance) {
        min_distance = distance;
        nearest_index = static_cast<int>(i);
      }
    }
  }

  // Publish only the next waypoints from the nearest one (same as navigation
  // goal)
  if (nearest_index >= 0 && !poses_.empty()) {
    for (int i = 0; i < number_of_waypoints_; ++i) {
      int idx = (nearest_index + i) % poses_.size();
      waypoint_array.poses.push_back(poses_[idx].pose);
    }
  } else {
    // Fallback: publish all waypoints if we can't determine position
    for (const auto & pose : poses_) {
      waypoint_array.poses.push_back(pose.pose);
    }
  }

  waypoint_array_publisher_->publish(waypoint_array);
  RCLCPP_DEBUG(
    this->get_logger(),
    "Published waypoint array with %zu waypoints (starting from index %d)",
    waypoint_array.poses.size(), nearest_index);
}

geometry_msgs::msg::PoseStamped WaypointPublisher::get_robot_pose()
{
  geometry_msgs::msg::PoseStamped robot_pose;

  try {
    // Get transform from map to base_link
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

    // Convert transform to pose
    robot_pose.header = transform_stamped.header;
    robot_pose.pose.position.x = transform_stamped.transform.translation.x;
    robot_pose.pose.position.y = transform_stamped.transform.translation.y;
    robot_pose.pose.position.z = transform_stamped.transform.translation.z;
    robot_pose.pose.orientation = transform_stamped.transform.rotation;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
    // Return empty pose on failure
    robot_pose.header.frame_id = "map";
    robot_pose.header.stamp = this->get_clock()->now();
  }

  return robot_pose;
}

int WaypointPublisher::find_nearest_waypoint_with_orientation(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (poses_.empty() || robot_pose.header.frame_id.empty()) {
    return -1;
  }

  // Get robot's yaw angle
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  double min_distance = std::numeric_limits<double>::max();
  int best_index = -1;

  for (size_t i = 0; i < poses_.size(); ++i) {
    // Calculate distance to waypoint
    double dx = poses_[i].pose.position.x - robot_pose.pose.position.x;
    double dy = poses_[i].pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Get waypoint's yaw angle
    double waypoint_yaw = tf2::getYaw(poses_[i].pose.orientation);

    // Check if orientations are within ±90 degrees
    double angle_diff = std::abs(angle_difference(robot_yaw, waypoint_yaw));
    if (angle_diff <= M_PI / 2.0) {  // 90 degrees in radians
      if (distance < min_distance) {
        min_distance = distance;
        best_index = static_cast<int>(i);
      }
    }
  }

  return best_index;
}

double WaypointPublisher::normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double WaypointPublisher::angle_difference(double angle1, double angle2)
{
  return normalize_angle(angle1 - angle2);
}
