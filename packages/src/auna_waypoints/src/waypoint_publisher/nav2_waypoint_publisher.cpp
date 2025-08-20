#include "auna_physical/nav2_waypoint_publisher.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <thread>

Nav2WaypointPublisher::Nav2WaypointPublisher()
: Node("nav2_waypoint_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // Start timer with a delay to allow Nav2 to initialize
  timer_ = this->create_wall_timer(std::chrono::milliseconds(12000), [this]() {
    // After first callback, switch to faster interval
    timer_->cancel();
    timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { timer_callback(); });
    timer_callback();
  });

  // Declare and get parameters
  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter<std::string>("namespace", namespace_);
  this->declare_parameter<std::string>("waypoint_file", "");
  this->get_parameter<std::string>("waypoint_file", waypoint_file_);

  // Create action client with proper namespace (keeping our improvement)
  std::string action_name =
    namespace_.empty() ? "navigate_through_poses" : namespace_ + "/navigate_through_poses";
  this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this, action_name);

  this->pose_array_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("waypoint_poses", 10);

  // Load waypoints from YAML file
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

        // Extract position information
        pose.pose.position.x = waypoint["position"]["x"].as<double>();
        pose.pose.position.y = waypoint["position"]["y"].as<double>();
        pose.pose.position.z = waypoint["position"]["z"].as<double>();

        // Extract orientation information (quaternion)
        pose.pose.orientation.x = waypoint["orientation"]["x"].as<double>();
        pose.pose.orientation.y = waypoint["orientation"]["y"].as<double>();
        pose.pose.orientation.z = waypoint["orientation"]["z"].as<double>();
        pose.pose.orientation.w = waypoint["orientation"]["w"].as<double>();

        poses_.push_back(pose);
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load waypoints from '%s': %s", waypoint_file_.c_str(),
      ex.what());
  }

  number_of_waypoints_ = poses_.size();
  RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoint(s)", poses_.size());

  // Debug: Log all loaded waypoints
  for (size_t i = 0; i < poses_.size(); ++i) {
    const auto & pos = poses_[i].pose.position;
    RCLCPP_INFO(
      this->get_logger(), "Loaded waypoint %zu: [%.6f, %.6f, %.6f]", i, pos.x, pos.y, pos.z);
  }

  // Original initialization: start from zero at first iteration
  current_pose_index_ = -number_of_waypoints_;
  remaining_number_of_poses_ = 0;
  goal_rejection_count_ = 0;
  waiting_for_retry_ = false;
}

void Nav2WaypointPublisher::timer_callback()
{
  // Publish pose array for visualization
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = this->now();
  for (const auto & pose_stamped : poses_) {
    pose_array.poses.push_back(pose_stamped.pose);
  }
  pose_array_publisher_->publish(pose_array);

  // Original trigger condition: when less than half poses remain
  // But don't interfere if we're waiting for a retry
  if (!waiting_for_retry_ && remaining_number_of_poses_ < number_of_waypoints_ * 0.5) {
    RCLCPP_INFO(
      this->get_logger(), "Timer triggered waypoint publishing (remaining: %d)",
      remaining_number_of_poses_);
    publish_waypoints();
  } else if (waiting_for_retry_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for retry, skipping timer trigger");
  }
}

void Nav2WaypointPublisher::publish_waypoints()
{
  // Check if action server is available
  if (!action_server_available_) {
    // On first attempt, wait longer for action server
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(
        this->get_logger(),
        "NavigateThroughPoses action server not available after 5s, will retry...");
      return;
    } else {
      action_server_available_ = true;
      RCLCPP_INFO(this->get_logger(), "NavigateThroughPoses action server is now available!");
    }
  } else {
    // Quick check for subsequent calls
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_WARN(
        this->get_logger(),
        "NavigateThroughPoses action server temporarily unavailable, waiting...");
      action_server_available_ = false;
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Publishing waypoints (loaded=%zu)", poses_.size());

  // Ensure we have waypoints to publish
  if (poses_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No waypoints loaded");
    return;
  }

  // Original index calculation
  current_pose_index_ =
    (current_pose_index_ + number_of_waypoints_ - remaining_number_of_poses_) % poses_.size();

  // Create goal with all waypoints, starting from current index (original approach)
  auto goal_msg = NavigateThroughPoses::Goal();
  for (int i = current_pose_index_; i < current_pose_index_ + number_of_waypoints_; i++) {
    auto pose = poses_[i % poses_.size()];
    // Set current timestamp for each pose
    pose.header.stamp = this->now();
    goal_msg.poses.push_back(pose);
  }

  // Log summary of the goal poses
  if (!goal_msg.poses.empty()) {
    const auto & p0 = goal_msg.poses.front().pose.position;
    const auto & pN = goal_msg.poses.back().pose.position;
    RCLCPP_INFO(
      this->get_logger(),
      "Sending NavigateThroughPoses goal: poses=%zu, first=[%.3f, %.3f], last=[%.3f, %.3f]",
      goal_msg.poses.size(), p0.x, p0.y, pN.x, pN.y);
  }

  // Setup callbacks and send goal
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
    goal_rejection_count_++;

    if (goal_rejection_count_ <= MAX_GOAL_REJECTIONS) {
      // Exponential backoff: 5, 10, 15, 20, 25 seconds
      int retry_delay = goal_rejection_count_ * 5;
      RCLCPP_WARN(
        this->get_logger(),
        "Goal rejected (attempt %d/%d) - Nav2 still initializing, retrying in %d seconds",
        goal_rejection_count_, MAX_GOAL_REJECTIONS, retry_delay);

      // Set retry flag to prevent timer interference
      waiting_for_retry_ = true;

      // Use std::thread but ensure it's properly managed
      std::thread retry_thread([this, retry_delay]() {
        std::this_thread::sleep_for(std::chrono::seconds(retry_delay));
        RCLCPP_INFO(this->get_logger(), "Retrying waypoint goal after server rejection...");
        waiting_for_retry_ = false;
        publish_waypoints();
      });
      retry_thread.detach();

    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Goal rejected %d times - Nav2 may not be properly configured. Stopping retry attempts.",
        goal_rejection_count_);
      waiting_for_retry_ = false;  // Stop blocking timer
    }
  } else {
    // Success! Reset counter and proceed
    goal_rejection_count_ = 0;
    waiting_for_retry_ = false;
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void Nav2WaypointPublisher::feedback_callback(
  GoalHandleNavigateThroughPoses::SharedPtr,
  const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
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
      RCLCPP_INFO(this->get_logger(), "Batch SUCCEEDED");

      // Move to next batch
      current_pose_index_ += remaining_number_of_poses_;

      // If we've reached the end, wrap around to start
      if (current_pose_index_ >= number_of_waypoints_) {
        current_pose_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "Completed all waypoints, starting over");
      }

      // Wait a moment before sending next batch
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      publish_waypoints();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Batch was ABORTED - retrying in 5 seconds");
      // Retry the same batch after a delay
      std::this_thread::sleep_for(std::chrono::seconds(5));
      publish_waypoints();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Batch was CANCELED");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code (%d)", static_cast<int>(result.code));
      break;
  }
}