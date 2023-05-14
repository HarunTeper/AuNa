#include "auna_mqtt/mqtt_waypoint_receiver.hpp"

MQTTWaypointReceiver::MQTTWaypointReceiver() : Node("mqtt_waypoint_receiver")
{
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this](){timer_callback();});

    this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this,"navigate_through_poses");
}

void MQTTWaypointReceiver::timer_callback() 
{
    timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = NavigateThroughPoses::Goal();

    //create a vector of poses in a square with 2 m sides
    std::vector<geometry_msgs::msg::PoseStamped> poses;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 2;
    pose.pose.position.y = 2;
    pose.pose.orientation.w = 1;
    poses.push_back(pose);

    pose.pose.position.x = 2;
    pose.pose.position.y = -2;
    pose.pose.orientation.w = 1;
    poses.push_back(pose);

    pose.pose.position.x = -2;
    pose.pose.position.y = -2;
    pose.pose.orientation.w = 1;
    poses.push_back(pose);

    pose.pose.position.x = -2;
    pose.pose.position.y = 2;
    pose.pose.orientation.w = 1;
    poses.push_back(pose);

    goal_msg.poses = poses;

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MQTTWaypointReceiver::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&MQTTWaypointReceiver::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&MQTTWaypointReceiver::result_callback, this, std::placeholders::_1);
    
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void MQTTWaypointReceiver::goal_response_callback(std::shared_future<GoalHandleNavigateThroughPoses::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void MQTTWaypointReceiver::feedback_callback(
  GoalHandleNavigateThroughPoses::SharedPtr,
  const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback: %i", feedback->navigation_time.sec);
}

void MQTTWaypointReceiver::result_callback(const GoalHandleNavigateThroughPoses::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  RCLCPP_INFO(this->get_logger(), "Navigation finished");
  rclcpp::shutdown();
}