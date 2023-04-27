#include "auna_cacc/cacc_controller.hpp"

CaccController::CaccController() : Node("cacc_controller")
{
    sub_cam_ = this->create_subscription<auna_its_msgs::msg::CAM>("cam_filtered", 2, [this](const auna_its_msgs::msg::CAM::SharedPtr msg){this->cam_callback(msg);});
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg){this->odom_callback(msg);});
    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this](){this->timer_callback();});

    pub_x_lookahead_point_ = this->create_publisher<std_msgs::msg::Float64>("cacc/x_lookahead_point", 2);
    pub_y_lookahead_point_ = this->create_publisher<std_msgs::msg::Float64>("cacc/y_lookahead_point", 2);

    this->declare_parameter("standstill_distance", 1.0);
    this->declare_parameter("time_gap", 1.0);
    this->declare_parameter("wheelbase", 0.32);
    this->declare_parameter("kp", 1.0);
    this->declare_parameter("kd", 1.0);
    this->declare_parameter("max_velocity", 1.0);

    standstill_distance_ = this->get_parameter("standstill_distance").as_double();
    time_gap_ = this->get_parameter("time_gap").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();

    sub_standstill_distance_ = this->create_subscription<std_msgs::msg::Float64>("cacc/standstill_distance", 2, [this](const std_msgs::msg::Float64::SharedPtr msg){this->standstill_distance_ = msg->data;});
    sub_time_gap_ = this->create_subscription<std_msgs::msg::Float64>("cacc/time_gap", 2, [this](const std_msgs::msg::Float64::SharedPtr msg){this->time_gap_ = msg->data;});
}

void CaccController::cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg)
{
    cam_x_ = msg->x;
    cam_y_ = msg->y;
    cam_velocity_ = msg->v;

    if (last_cam_msg_ == nullptr)
    {
        cam_acceleration_ = cam_velocity_;
    }
    else
    {
        cam_acceleration_ = (cam_velocity_ - last_cam_velocity_) / (msg->header.stamp.sec - last_cam_msg_->header.stamp.sec + (msg->header.stamp.nanosec - last_cam_msg_->header.stamp.nanosec) / 1000000000.0);
    }

    last_cam_msg_ = msg;

    last_cam_velocity_ = cam_velocity_;

    cam_yaw_ = msg->theta;
    cam_yaw_rate_ = msg->thetadot;

    if (cam_velocity_ == 0)
    {
        cam_curvature_ = 0;
    }
    else
    {
        cam_curvature_ = cam_yaw_rate_ / cam_velocity_;
    }
}

void CaccController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_velocity_ = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2)) * msg->twist.twist.linear.x / abs(msg->twist.twist.linear.x);
    if (last_odom_msg_ == nullptr)
    {
        odom_acceleration_ = odom_velocity_;
    }
    else
    {
        odom_acceleration_ = (odom_velocity_ - last_odom_velocity_) / (msg->header.stamp.sec - last_odom_msg_->header.stamp.sec + (msg->header.stamp.nanosec - last_odom_msg_->header.stamp.nanosec) / 1000000000.0);
    }
    last_odom_msg_ = msg;
    last_odom_velocity_ = odom_velocity_;
    odom_yaw_rate_ = msg->twist.twist.angular.z;

    if (odom_velocity_ == 0)
    {
        odom_curvature_ = 0;
    }
    else
    {
        odom_curvature_ = odom_yaw_rate_ / odom_velocity_;
    }

}

void CaccController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;

    q_.setW(msg->pose.orientation.w);
    q_.setX(msg->pose.orientation.x);
    q_.setY(msg->pose.orientation.y);
    q_.setZ(msg->pose.orientation.z);
    q_.normalize();
    m_.setRotation(q_);
    m_.getRPY(roll_, pitch_, yaw_);
    pose_yaw_ = yaw_;

}

void CaccController::standstill_distance_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    standstill_distance_ = msg->data;
}

void CaccController::time_gap_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    time_gap_ = msg->data;
}

void CaccController::timer_callback()
{
    //print hello

    if(cam_curvature_ <= 0.01 && cam_curvature_ >= -0.01){
        s_ = 0.5*pow(standstill_distance_+time_gap_*odom_velocity_, 2)*cam_curvature_-0.125*pow(standstill_distance_+time_gap_*odom_velocity_, 4)*pow(cam_curvature_, 3);
    }
    else{
        s_ = (-1 + sqrt(1 + pow(cam_curvature_, 2) * pow(standstill_distance_ + time_gap_ * odom_velocity_, 2))) / cam_curvature_;
    }
    alpha_ = atan(cam_curvature_ * (standstill_distance_ + time_gap_ * odom_velocity_));

    x_lookahead_point_ = cam_x_+s_*sin(cam_yaw_);
    y_lookahead_point_ = cam_y_-s_*cos(cam_yaw_);

    std_msgs::msg::Float64 x_lookahead_point_msg;
    x_lookahead_point_msg.data = x_lookahead_point_;
    pub_x_lookahead_point_->publish(x_lookahead_point_msg);

    std_msgs::msg::Float64 y_lookahead_point_msg;
    y_lookahead_point_msg.data = y_lookahead_point_;
    pub_y_lookahead_point_->publish(y_lookahead_point_msg);

    z_1_ = cam_x_ - pose_x_ + s_ * sin(cam_yaw_) - (standstill_distance_+time_gap_*odom_velocity_) * cos(pose_yaw_);
    z_2_ = cam_y_ - pose_y_ - s_ * cos(cam_yaw_) - (standstill_distance_+time_gap_*odom_velocity_) * sin(pose_yaw_);
    z_3_ = cam_velocity_ * cos(cam_yaw_) - odom_velocity_ * cos(pose_yaw_ + alpha_);
    z_4_ = cam_velocity_ * sin(cam_yaw_) - odom_velocity_ * sin(pose_yaw_ + alpha_);

    invGam_Det_ = (standstill_distance_+time_gap_*odom_velocity_)*(time_gap_-time_gap_*sin(s_)*sin(cam_yaw_-pose_yaw_));

    invGam_1_ = ((standstill_distance_+time_gap_*odom_velocity_)*cos(pose_yaw_))/invGam_Det_;
    invGam_2_ = ((standstill_distance_+time_gap_*odom_velocity_)*sin(pose_yaw_))/invGam_Det_;
    invGam_3_ = (-time_gap_*sin(pose_yaw_)-time_gap_*sin(s_)*cos(cam_yaw_))/invGam_Det_;
    invGam_4_ = (time_gap_*cos(pose_yaw_)-time_gap_*sin(s_)*sin(cam_yaw_))/invGam_Det_;

    inP_1_ = (z_1_ * kp_) + ( cos(alpha_)*z_3_+sin(alpha_)*z_4_ ) + ( (1-cos(alpha_))*cam_velocity_*cos(cam_yaw_)-sin(alpha_)*cam_velocity_*sin(cam_yaw_) ) + ( cos(cam_yaw_)*s_*cam_yaw_rate_ );
    inP_2_ = (z_2_ * kd_) + ( sin(alpha_)*z_3_+cos(alpha_)*z_4_ ) + ( sin(alpha_)*cam_velocity_*cos(cam_yaw_)+(1-cos(alpha_))*cam_velocity_*sin(cam_yaw_) ) + ( sin(cam_yaw_)*s_*cam_yaw_rate_ );

    a_ = invGam_1_ * inP_1_ + invGam_2_ * inP_2_;
    w_ = invGam_3_ * inP_1_ + invGam_4_ * inP_2_;

    double dt_ = (rclcpp::Clock().now() - last_time_).seconds();
    last_time_ = rclcpp::Clock().now();

    v_ = last_velocity_ + a_ * dt_;

    if(v_ > max_velocity_){
        v_ = max_velocity_;
    }
    else if(v_ < 0.01){
        v_ = 0.0;
    }

    if(v_ == 0.0){
        w_ = 0.0;
    }
    else{
        w_ = atan(w_*wheelbase_/v_);
    }
    last_velocity_ = v_;

    //create a twist message
    geometry_msgs::msg::Twist twist_msg;

    //set the linear and angular velocities
    twist_msg.linear.x = v_;
    twist_msg.angular.z = w_;

    //publish the twist message
    pub_cmd_vel->publish(twist_msg);
}

