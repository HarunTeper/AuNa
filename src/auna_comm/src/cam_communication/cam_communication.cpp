#include "auna_comm/cam_communication.hpp"
#include "auna_its_msgs/msg/cam.hpp"

CamCommunication::CamCommunication() : Node("cam_communication")
{
    this->declare_parameter("filter_index", 0);
    this->declare_parameter("robot_index", 0);

    this->filter_index_ = this->get_parameter("filter_index").as_int();
    this->robot_index_ = this->get_parameter("robot_index").as_int();

    cam_filtered_publisher_ = this->create_publisher<auna_its_msgs::msg::CAM>("cam_filtered", 2);
    cam_publisher_ = this->create_publisher<auna_its_msgs::msg::CAM>("/cam", 2);
    cam_subscriber_ = this->create_subscription<auna_its_msgs::msg::CAM>("/cam", 2, [this](auna_its_msgs::msg::CAM::SharedPtr msg) -> void { this->cam_callback(msg); });
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() -> void { this->timer_callback(); });
    
    this->pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("global_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){pose_callback(msg);});
    this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg){odom_callback(msg);});

    last_cam_msg_time_ = this->now();
}

void CamCommunication::cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg)
{   
    if (msg->robot_name == std::to_string(this->filter_index_))
    {
        cam_filtered_publisher_->publish(*msg);
    }
}

void CamCommunication::timer_callback()
{
    if (this->now() - last_cam_msg_time_ >= std::chrono::milliseconds(1000))
    {
        publish_cam_msg("timeout");
    }
    else
    {
        if (fabs(this->speed_ - last_cam_msg_.v > 0.05))
        {
            publish_cam_msg("speed");
        }
        else if (sqrt(pow(this->longitude_ - last_cam_msg_.x, 2) + pow(this->latitude_ - last_cam_msg_.y, 2)) > 0.4)
        {
            publish_cam_msg("position");
        }
        else if (fabs(this->heading_ - last_cam_msg_.theta) > 4 * M_PI / 180)
        {
            publish_cam_msg("heading");
        }
    }
}

void CamCommunication::publish_cam_msg(std::string frame_id)
{
    auto msg = auna_its_msgs::msg::CAM();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id;
    msg.robot_name = std::to_string(this->robot_index_);
    msg.x = this->longitude_;
    msg.y = this->latitude_;
    msg.z = this->altitude_;
    msg.theta = this->heading_;
    msg.thetadot = this->yaw_rate_;
    msg.drive_direction = this->drive_direction_;
    msg.v = this->speed_;
    msg.vdot = this->acceleration_;
    msg.curv = this->curvature_;
    msg.vehicle_length = 0.4;
    msg.vehicle_width = 0.2;

    //publish message
    cam_publisher_->publish(msg);

    last_cam_msg_ = msg;
    last_cam_msg_time_ = msg.header.stamp;
}

void CamCommunication::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    this->speed_ = sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2));

    if (last_odom_msg_ != nullptr)
    {
        this->old_speed_ = sqrt(pow(last_odom_msg_->twist.twist.linear.x,2)+pow(last_odom_msg_->twist.twist.linear.y,2));//absolute speed without direction
        this->acceleration_ = (this->speed_ - this->old_speed_) / (msg->header.stamp.sec - last_odom_msg_->header.stamp.sec + (msg->header.stamp.nanosec - last_odom_msg_->header.stamp.nanosec) / 1000000000.0);
    }
    else
    {
        this->acceleration_ = 0;
    }
    this->yaw_rate_ = msg->twist.twist.angular.z;

    if (this->speed_ == 0)
    {
        this->curvature_ = 0;
    }
    else
    {
        this->curvature_ = this->yaw_rate_ / this->speed_;
    }

    if (msg->twist.twist.linear.x > 0)
    {
        this->drive_direction_ = 1;
    }
    else if (msg->twist.twist.linear.x < 0)
    {
        this->drive_direction_ = -1;
    }
    else
    {
        this->drive_direction_ = 0;
    }

    this->last_odom_msg_ = msg;
}

void CamCommunication::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->longitude_ = msg->pose.position.x;
    this->latitude_ = msg->pose.position.y;
    this->altitude_ = msg->pose.position.z;

    tf2::Quaternion quat(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    quat.normalize();
    tf2::Matrix3x3 matrix(quat);
    tf2Scalar roll;
    tf2Scalar pitch;
    tf2Scalar yaw;
    matrix.getRPY(roll, pitch, yaw);

    this->heading_ = yaw;
}