#include "auna_physical/nav2_waypoint_publisher.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

Nav2WaypointPublisher::Nav2WaypointPublisher() : Node("nav2_waypoint_publisher")
{
  // Create timer for publishing pose array and managing waypoint cycles
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { timer_callback(); });

  // Declare and get parameters
  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter<std::string>("namespace", namespace_);
  this->declare_parameter<std::string>("waypoint_file", "");
  this->get_parameter<std::string>("waypoint_file", waypoint_file_);

  // Create action client with proper namespace
  std::string action_name =
    namespace_.empty() ? "navigate_through_poses" : namespace_ + "/navigate_through_poses";
  this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this, action_name);

  // Create publisher for pose array visualization
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

  number_of_waypoints_ = poses_.size();
  RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoint(s)", poses_.size());

  // Initialize pose index to start from beginning
  current_pose_index_ = 0;
  remaining_number_of_poses_ = 0;
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

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000, "Published PoseArray with %zu poses",
    pose_array.poses.size());

  // Trigger waypoint publishing when less than half poses remain
  if (remaining_number_of_poses_ < number_of_waypoints_ * 0.5) {
    publish_waypoints();
  }
}

void Nav2WaypointPublisher::publish_waypoints()
{
  RCLCPP_INFO(this->get_logger(), "Publishing waypoints (loaded=%zu)", poses_.size());

  // Ensure we have waypoints to publish
  if (poses_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No waypoints loaded");
    return;
  }

  // Ensure the Nav2 action server is available
  if (!client_ptr_ || !client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(
      this->get_logger(), "NavigateThroughPoses action server not available yet; will retry");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "NavigateThroughPoses action server is available");

  // Update current pose index for next batch
  current_pose_index_ =
    (current_pose_index_ + number_of_waypoints_ - remaining_number_of_poses_) % poses_.size();

  // Create goal with all waypoints, starting from current index
  auto goal_msg = NavigateThroughPoses::Goal();
  for (int i = 0; i < number_of_waypoints_; i++) {
    auto pose = poses_[(current_pose_index_ + i) % poses_.size()];
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

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  remaining_number_of_poses_ = number_of_waypoints_;
}

void Nav2WaypointPublisher::goal_response_callback(
  GoalHandleNavigateThroughPoses::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
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
      RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED - publishing next batch of waypoints");
      publish_waypoints();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was ABORTED");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was CANCELED");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code (%d)", static_cast<int>(result.code));
      break;
  }
}