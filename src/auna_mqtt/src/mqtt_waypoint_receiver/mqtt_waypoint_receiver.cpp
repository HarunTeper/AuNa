#include "auna_mqtt/mqtt_waypoint_receiver.hpp"

MQTTWaypointReceiver::MQTTWaypointReceiver() : Node("mqtt_waypoint_receiver"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this](){timer_callback();});

    this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this,"navigate_through_poses");

    this->declare_parameter<std::string>("namespace", "");
    this->get_parameter<std::string>("namespace", namespace_);
}

void MQTTWaypointReceiver::timer_callback() 
{
    timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        if (namespace_ != "") {
            transformStamped = tf_buffer_.lookupTransform("map", namespace_+"/base_link", tf2::TimePointZero);
        } else {
            transformStamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        timer_->reset();
        return;
    }

    auto goal_msg = NavigateThroughPoses::Goal();

    std::vector<geometry_msgs::msg::PoseStamped> poses;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = transformStamped.transform.translation.x;
    pose.pose.position.y = transformStamped.transform.translation.y;
    pose.pose.position.z = transformStamped.transform.translation.z;
    pose.pose.orientation.x = transformStamped.transform.rotation.x;
    pose.pose.orientation.y = transformStamped.transform.rotation.y;
    pose.pose.orientation.z = transformStamped.transform.rotation.z;
    pose.pose.orientation.w = transformStamped.transform.rotation.w;
    poses.push_back(pose);

    double distance = 3.0;
    pose.pose.position.x = distance;
    pose.pose.position.y = distance;
    double heading = atan2(pose.pose.position.y - poses[0].pose.position.y, pose.pose.position.x - poses[0].pose.position.x);
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), heading));
    poses.push_back(pose);
    pose.pose.position.x = distance;
    pose.pose.position.y = -distance;
    heading = atan2(pose.pose.position.y - poses[1].pose.position.y, pose.pose.position.x - poses[0].pose.position.x);
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), heading));
    poses.push_back(pose);
    pose.pose.position.x = -distance;
    pose.pose.position.y = -distance;
    heading = atan2(pose.pose.position.y - poses[2].pose.position.y, pose.pose.position.x - poses[0].pose.position.x);
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), heading));
    poses.push_back(pose);
    pose.pose.position.x = -distance;
    pose.pose.position.y = distance;
    heading = atan2(pose.pose.position.y - poses[3].pose.position.y, pose.pose.position.x - poses[0].pose.position.x);
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), heading));
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
  timer_->reset();
}
