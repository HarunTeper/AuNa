#include "auna_physical/nav2_waypoint_publisher.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

#include <sstream>

Nav2WaypointPublisher::Nav2WaypointPublisher()
: Node("nav2_waypoint_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { timer_callback(); });

  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter<std::string>("namespace", namespace_);

  // Create action client with proper namespace
  std::string action_name =
    namespace_.empty() ? "navigate_through_poses" : namespace_ + "/navigate_through_poses";
  this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this, action_name);

  this->pose_array_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("waypoint_poses", 10);
  this->declare_parameter<std::string>("waypoint_file", "");
  this->get_parameter<std::string>("waypoint_file", waypoint_file_);
  this->declare_parameter<bool>("wait_for_bt_active", false);
  this->get_parameter<bool>("wait_for_bt_active", wait_for_bt_active_);

  // Load waypoints from YAML with error handling
  try {
    YAML::Node waypoints = YAML::LoadFile(waypoint_file_);
    if (!waypoints || !waypoints.IsSequence()) {
      RCLCPP_ERROR(
        this->get_logger(), "Waypoint file '%s' is empty or not a sequence",
        waypoint_file_.c_str());
    } else {
      for (const auto & waypoint : waypoints) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();  // Set current timestamp
        pose.pose.position.x = waypoint["x"].as<float>();
        pose.pose.position.y = waypoint["y"].as<float>();
        pose.pose.position.z = 0;

        tf2::Quaternion q;
        q.setRPY(0, 0, waypoint["yaw"].as<double>());
        pose.pose.orientation = tf2::toMsg(q);

        poses_.push_back(pose);
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load waypoints from '%s': %s", waypoint_file_.c_str(),
      ex.what());
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoint(s)", poses_.size());
  number_of_waypoints_ = poses_.size();
  if (!poses_.empty()) {
    const auto & first = poses_.front().pose.position;
    const auto & last = poses_.back().pose.position;
    RCLCPP_DEBUG(
      this->get_logger(), "First waypoint: [%.3f, %.3f], Last waypoint: [%.3f, %.3f]", first.x,
      first.y, last.x, last.y);
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

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Published PoseArray with %zu poses (stamp=%d.%09u), subscribers=%zu", pose_array.poses.size(),
    pose_array.header.stamp.sec, pose_array.header.stamp.nanosec,
    pose_array_publisher_ ? pose_array_publisher_->get_subscription_count() : 0);

  if (remaining_number_of_poses_ < number_of_waypoints_ * 0.5) {
    RCLCPP_DEBUG(
      this->get_logger(), "Triggering publish_waypoints: remaining=%d, batch=%d",
      remaining_number_of_poses_, number_of_waypoints_);
    publish_waypoints();
  }
}

void Nav2WaypointPublisher::publish_waypoints()
{
  RCLCPP_INFO(this->get_logger(), "Publishing waypoints (loaded=%zu)", poses_.size());

  // Ensure we have waypoints to publish
  if (poses_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No waypoints loaded yet");
    return;
  }

  // Ensure the Nav2 action server is available before sending a goal
  if (!client_ptr_ || !client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "NavigateThroughPoses action server not available yet; will retry");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "NavigateThroughPoses action server is available");

  int prev_index = current_pose_index_;
  current_pose_index_ =
    (current_pose_index_ + number_of_waypoints_ - remaining_number_of_poses_) % poses_.size();
  RCLCPP_DEBUG(
    this->get_logger(),
    "Selecting goal batch: prev_index=%d -> start_index=%d, batch_size=%d, remaining_before=%d",
    prev_index, current_pose_index_, number_of_waypoints_, remaining_number_of_poses_);

  auto goal_msg = NavigateThroughPoses::Goal();
  for (int i = current_pose_index_; i < current_pose_index_ + number_of_waypoints_; i++) {
    auto pose = poses_[i % poses_.size()];
    // Update timestamp to current time for each pose
    pose.header.stamp = this->now();
    goal_msg.poses.push_back(pose);
  }

  // Log a concise summary of the goal poses
  if (!goal_msg.poses.empty()) {
    const auto & p0 = goal_msg.poses.front().pose.position;
    const auto & pN = goal_msg.poses.back().pose.position;
    RCLCPP_INFO(
      this->get_logger(),
      "Sending NavigateThroughPoses goal: poses=%zu, first=[%.3f, %.3f], last=[%.3f, %.3f]",
      goal_msg.poses.size(), p0.x, p0.y, pN.x, pN.y);
  }

  for (size_t i = 0; i < goal_msg.poses.size(); ++i) {
    tf2::Quaternion q;
    tf2::fromMsg(goal_msg.poses[i].pose.orientation, q);
    RCLCPP_INFO(
      this->get_logger(), "Goal Pose %zu: [%.3f, %.3f, %.3f]", i, goal_msg.poses[i].pose.position.x,
      goal_msg.poses[i].pose.position.y, q.getAngle());
  }

  auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Nav2WaypointPublisher::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &Nav2WaypointPublisher::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&Nav2WaypointPublisher::result_callback, this, std::placeholders::_1);

  RCLCPP_DEBUG(this->get_logger(), "Calling async_send_goal()");
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  remaining_number_of_poses_ = number_of_waypoints_;
}

bool Nav2WaypointPublisher::is_bt_navigator_active()
{
  if (!bt_get_state_client_) return false;
  if (!bt_get_state_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_DEBUG(
      this->get_logger(), "Waiting for service: %s", bt_get_state_client_->get_service_name());
    return false;
  }
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = bt_get_state_client_->async_send_request(request);
  auto status = future.wait_for(std::chrono::seconds(2));
  if (status != std::future_status::ready) {
    RCLCPP_DEBUG(
      this->get_logger(), "GetState request timed out for service: %s",
      bt_get_state_client_->get_service_name());
    return false;
  }
  auto resp = future.get();
  if (!resp) {
    RCLCPP_DEBUG(
      this->get_logger(), "GetState returned null response for service: %s",
      bt_get_state_client_->get_service_name());
    return false;
  }
  RCLCPP_DEBUG(
    this->get_logger(), "bt_navigator state id=%u label=%s", resp->current_state.id,
    resp->current_state.label.c_str());
  return resp->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

void Nav2WaypointPublisher::goal_response_callback(
  GoalHandleNavigateThroughPoses::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
  // Don't call publish_waypoints() here to avoid infinite recursion
}

void Nav2WaypointPublisher::feedback_callback(
  GoalHandleNavigateThroughPoses::SharedPtr,
  const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
  // RCLCPP_INFO(this->get_logger(), "Received feedback: %i", feedback->number_of_poses_remaining);
  remaining_number_of_poses_ = feedback->number_of_poses_remaining;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000, "Feedback: %d poses remaining",
    remaining_number_of_poses_);
}

void Nav2WaypointPublisher::result_callback(
  const GoalHandleNavigateThroughPoses::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED");
      // Only publish new waypoints after successful completion
      publish_waypoints();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was ABORTED");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was CANCELED");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code (%d)", static_cast<int>(result.code));
      return;
  }
  RCLCPP_INFO(this->get_logger(), "Navigation finished");
}