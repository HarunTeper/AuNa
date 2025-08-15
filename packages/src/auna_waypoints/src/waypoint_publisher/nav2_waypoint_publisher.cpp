#include "auna_physical/nav2_waypoint_publisher.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

Nav2WaypointPublisher::Nav2WaypointPublisher()
: Node("nav2_waypoint_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { timer_callback(); });

  this->client_ptr_ =
    rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");

  this->pose_array_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("waypoint_poses", 10);

  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter<std::string>("namespace", namespace_);
  this->declare_parameter<std::string>("waypoint_file", "");
  this->get_parameter<std::string>("waypoint_file", waypoint_file_);

  YAML::Node waypoints = YAML::LoadFile(waypoint_file_);
  for (const auto & waypoint : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = waypoint["x"].as<float>();
    pose.pose.position.y = waypoint["y"].as<float>();
    pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, waypoint["yaw"].as<double>());
    pose.pose.orientation = tf2::toMsg(q);

    poses_.push_back(pose);
  }

  // to start from zero at first iteration
  current_pose_index_ = -number_of_waypoints_;
}

void Nav2WaypointPublisher::timer_callback()
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = this->now();
  for (const auto & pose_stamped : poses_) {
    pose_array.poses.push_back(pose_stamped.pose);
  }
  pose_array_publisher_->publish(pose_array);

  if (remaining_number_of_poses_ < number_of_waypoints_ * 0.5) {
    publish_waypoints();
  }
}

void Nav2WaypointPublisher::publish_waypoints()
{
  RCLCPP_INFO(this->get_logger(), "Publishing waypoints");

  current_pose_index_ =
    (current_pose_index_ + number_of_waypoints_ - remaining_number_of_poses_) % poses_.size();

  auto goal_msg = NavigateThroughPoses::Goal();
  for (int i = current_pose_index_; i < current_pose_index_ + number_of_waypoints_; i++) {
    goal_msg.poses.push_back(poses_[i % poses_.size()]);
  }

  auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Nav2WaypointPublisher::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &Nav2WaypointPublisher::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&Nav2WaypointPublisher::result_callback, this, std::placeholders::_1);

  RCLCPP_INFO(this->get_logger(), "Sending goal");
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  remaining_number_of_poses_ = number_of_waypoints_;
}

void Nav2WaypointPublisher::goal_response_callback(
  GoalHandleNavigateThroughPoses::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    publish_waypoints();
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void Nav2WaypointPublisher::feedback_callback(
  GoalHandleNavigateThroughPoses::SharedPtr,
  const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
  // RCLCPP_INFO(this->get_logger(), "Received feedback: %i", feedback->number_of_poses_remaining);
  remaining_number_of_poses_ = feedback->number_of_poses_remaining;
}

void Nav2WaypointPublisher::result_callback(
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