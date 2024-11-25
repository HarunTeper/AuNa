#include "auna_physical/waypoint_publisher.hpp"

WaypointPublisher::WaypointPublisher() : Node("waypoint_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this](){timer_callback();});

    this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this,"navigate_through_poses");

    this->declare_parameter<std::string>("namespace", "");
    this->get_parameter<std::string>("namespace", namespace_);
    this->declare_parameter<std::string>("waypoint_file", "");
    this->get_parameter<std::string>("waypoint_file", waypoint_file_);

    std::ifstream file(waypoint_file_, std::ifstream::in);
    std::string line;
    if(file.is_open()) {
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string x, y;
            std::getline(iss, x, ',');
            std::getline(iss, y, ',');

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = std::stof(x);
            pose.pose.position.y = std::stof(y);
            pose.pose.position.z = 0;

            double heading = 0;
            pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), heading));

            poses_.push_back(pose);
        }
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unable to open file");
        return;
    }
    file.close();
    for (uint i = 0; i < poses_.size(); i++) {
        if (i == 0) {
            double heading = atan2(poses_[i+1].pose.position.y - poses_[poses_.size()-1].pose.position.y, poses_[i+1].pose.position.x - poses_[poses_.size()-1].pose.position.x);
            poses_[i].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), heading));
        } else if (i == poses_.size()-1) {
            double heading = atan2(poses_[0].pose.position.y - poses_[i-1].pose.position.y, poses_[0].pose.position.x - poses_[i-1].pose.position.x);
            poses_[i].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), heading));
        } else {
            double heading = atan2(poses_[i+1].pose.position.y - poses_[i-1].pose.position.y, poses_[i+1].pose.position.x - poses_[i-1].pose.position.x);
            poses_[i].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), heading));
        }
    }

    // to start from zero at first iteration
    current_pose_index_ = -number_of_waypoints_;
}

void WaypointPublisher::timer_callback()
{
    if(remaining_number_of_poses_ < number_of_waypoints_*0.5){
        publish_waypoints();
    }
}

void WaypointPublisher::publish_waypoints(){

    RCLCPP_INFO(this->get_logger(), "Publishing waypoints");

    current_pose_index_ = (current_pose_index_ + number_of_waypoints_ - remaining_number_of_poses_) % poses_.size();

    auto goal_msg = NavigateThroughPoses::Goal();
    for (int i = current_pose_index_; i < current_pose_index_ + number_of_waypoints_; i++) {
        goal_msg.poses.push_back(poses_[i % poses_.size()]);
    }
    
    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&WaypointPublisher::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&WaypointPublisher::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&WaypointPublisher::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    remaining_number_of_poses_ = number_of_waypoints_;
}

void WaypointPublisher::goal_response_callback(std::shared_future<GoalHandleNavigateThroughPoses::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        publish_waypoints();
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void WaypointPublisher::feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
    // RCLCPP_INFO(this->get_logger(), "Received feedback: %i", feedback->number_of_poses_remaining);
    remaining_number_of_poses_ = feedback->number_of_poses_remaining;
}

void WaypointPublisher::result_callback(const GoalHandleNavigateThroughPoses::WrappedResult & result)
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