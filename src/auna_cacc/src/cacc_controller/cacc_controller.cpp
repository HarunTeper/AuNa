#include "auna_cacc/cacc_controller.hpp"

CaccController::CaccController() : Node("cacc_controller")
{
    sub_cam_ = this->create_subscription<auna_its_msgs::msg::CAM>("cam_filtered", 2, [this](const auna_its_msgs::msg::CAM::SharedPtr msg){this->cam_callback(msg);});
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg){this->odom_callback(msg);});
    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("global_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
    
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    pub_x_lookahead_point_ = this->create_publisher<std_msgs::msg::Float64>("cacc/lookahead/x", 1);
    pub_y_lookahead_point_ = this->create_publisher<std_msgs::msg::Float64>("cacc/lookahead/y", 1);

    client_set_standstill_distance_ = this->create_service<auna_msgs::srv::SetFloat64>("cacc/set_standstill_distance", [this](const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request, std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response){this->set_standstill_distance(request, response);});
    client_set_time_gap_ = this->create_service<auna_msgs::srv::SetFloat64>("cacc/set_time_gap", [this](const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request, std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response){this->set_time_gap(request, response);});
    client_set_cacc_enable_ = this->create_service<auna_msgs::srv::SetBool>("cacc/set_cacc_enable", [this](const std::shared_ptr<auna_msgs::srv::SetBool::Request> request, std::shared_ptr<auna_msgs::srv::SetBool::Response> response){this->set_cacc_enable(request, response);});

    this->declare_parameter("standstill_distance", 1.5);
    this->declare_parameter("time_gap", 0.5);
    this->declare_parameter("kp", 0.5);
    this->declare_parameter("kd", 0.5);
    this->declare_parameter("max_velocity", 1.0);
    this->declare_parameter("frequency", 50);
    this->declare_parameter("use_waypoints", false);
    this->declare_parameter("waypoint_file", "/home/$USER/waypoints.txt");
    this->declare_parameter("target_velocity", 1.0);

    standstill_distance_ = this->get_parameter("standstill_distance").as_double();
    time_gap_ = this->get_parameter("time_gap").as_double();
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    frequency_ = this->get_parameter("frequency").as_int();
    use_waypoints_ = this->get_parameter("use_waypoints").as_bool();
    target_velocity_ = this->get_parameter("target_velocity").as_double();

    dt_ = 1.0 / frequency_;
    auto_mode_ = false;
    auto_mode_ready_ = false;
    client_set_auto_mode_ = this->create_service<auna_msgs::srv::SetBool>("cacc/set_auto_mode", [this](const std::shared_ptr<auna_msgs::srv::SetBool::Request> request, std::shared_ptr<auna_msgs::srv::SetBool::Response> response){this->set_auto_mode(request, response);});

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / frequency_), [this](){this->timer_callback();});
    setup_timer_ = this->create_wall_timer(std::chrono::milliseconds(250), [this](){this->setup_timer_callback();});
    timer_->cancel();

    if (use_waypoints_)
    {
        read_waypoints_from_csv();
        closest_waypoint_index_ = 0;
    }
}

void CaccController::read_waypoints_from_csv()
{
    std::string file_path = this->get_parameter("waypoint_file").as_string();

    std::ifstream file(file_path, std::ifstream::in);
    std::string line;
    if (file.is_open())
    {
        waypoints_x_.clear(); // Clear the existing waypoints_x_ vector
        waypoints_y_.clear(); // Clear the existing waypoints_y_ vector

        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string x, y;
            std::getline(iss, x, ',');
            std::getline(iss, y, ',');
            try
            {
                double x_value = std::stod(x);
                double y_value = std::stod(y);

                waypoints_x_.push_back(x_value);
                waypoints_y_.push_back(y_value);
            }
            catch (const std::invalid_argument& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid waypoint format in CSV file: %s", file_path.c_str());
            }
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", file_path.c_str());
        return;
    }
    file.close();

    // Calculate yaw for each waypoint
    for (size_t i = 1; i < waypoints_x_.size() - 1; ++i)
    {
        double next_x = waypoints_x_[i + 1];
        double next_y = waypoints_y_[i + 1];

        double prev_x = waypoints_x_[i - 1];
        double prev_y = waypoints_y_[i - 1];

        double yaw = std::atan2(next_y - prev_y, next_x - prev_x);
        waypoints_yaw_.push_back(yaw);
    }
}

void CaccController::setup_timer_callback()
{
    // if all last messages exist, start timer
    if (last_cam_msg_ != nullptr && last_odom_msg_ != nullptr && last_pose_msg_ != nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "CACC controller started");
        timer_->reset();
        setup_timer_->cancel();
    }
    if (last_odom_msg_ != nullptr && last_pose_msg_ != nullptr)
    {
        if(!auto_mode_ready_){
            RCLCPP_INFO(this->get_logger(), "Auto mode ready");
            auto_mode_ready_ = true;
        }
    }
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
        double dt = msg->header.stamp.sec - last_cam_msg_->header.stamp.sec + (msg->header.stamp.nanosec - last_cam_msg_->header.stamp.nanosec) / 1e9;
        dt = std::min(dt, 0.1);
        cam_acceleration_ = (cam_velocity_ - last_cam_velocity_) / dt;
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
    odom_velocity_ = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2)) * ((msg->twist.twist.linear.x < 0) ? -1 : 1);
    if (last_odom_msg_ == nullptr)
    {
        odom_acceleration_ = odom_velocity_;
    }
    else
    {
        double dt = msg->header.stamp.sec - last_odom_msg_->header.stamp.sec + (msg->header.stamp.nanosec - last_odom_msg_->header.stamp.nanosec) / 1e9;
        dt = std::min(dt, 0.01);
        odom_acceleration_ = (odom_velocity_ - last_odom_velocity_) / dt;
        
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

    last_pose_msg_ = msg;
}

void CaccController::timer_callback()
{
    if(use_waypoints_){
        double closest_distance_squared = std::numeric_limits<double>::max();
        int num_waypoints = waypoints_x_.size();

        for (int i = closest_waypoint_index_; true; i = (i + 1) % num_waypoints)
        {
            double dx = waypoints_x_[i] - pose_x_;
            double dy = waypoints_y_[i] - pose_y_;
            double distance_squared = dx * dx + dy * dy;

            if (distance_squared > closest_distance_squared)
            {
                // Stop the loop if the distance is increasing
                break;
            }

            closest_distance_squared = distance_squared;
            closest_waypoint_index_ = i;
        }

        int target_waypoint_index_ = closest_waypoint_index_;
        if (auto_mode_)
        {
            // Calculate target waypoint index (the waypoint that is closest to the vehicle and within the time gap

            double target_distance = standstill_distance_ + time_gap_ * target_velocity_;


            for (int i = closest_waypoint_index_; true; i = (i + 1) % num_waypoints)
            {
                double dx = waypoints_x_[i] - pose_x_;
                double dy = waypoints_y_[i] - pose_y_;
                double distance_squared = dx * dx + dy * dy;

                if (distance_squared > target_distance * target_distance)
                {
                    // Stop the loop if the distance is increasing
                    break;
                }

                target_waypoint_index_ = i;
            }
        }

        // Calculate yaw difference between previous and next waypoints
        int curr_index = target_waypoint_index_;
        int next_index = (target_waypoint_index_ + 1) % num_waypoints;

        double current_yaw = waypoints_yaw_[curr_index];
        double next_yaw = waypoints_yaw_[next_index];
        double yaw_difference = next_yaw - current_yaw;

        double dx = waypoints_x_[next_index] - waypoints_x_[curr_index];
        double dy = waypoints_y_[next_index] - waypoints_y_[curr_index];
        double distance = std::hypot(dx, dy);

        // calculate the required time by dividing distance through cam_velocity_
        double required_time;
        if(auto_mode_){
            required_time = distance / target_velocity_;
        }
        else{
            required_time = distance / cam_velocity_;
        }

        // Use closest waypoint yaw
        cam_yaw_ = current_yaw;
        cam_yaw_rate_ = yaw_difference / required_time;

        //cam curvature depending on auto_mode
        if(auto_mode_){
            cam_curvature_ = cam_yaw_rate_ / target_velocity_;
        }
        else{
            if (cam_velocity_ == 0)
            {
                cam_curvature_ = 0;
            }
            else
            {
                cam_curvature_ = cam_yaw_rate_ / cam_velocity_;
            }
        }
    }

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

    last_time_ = rclcpp::Clock().now();

    v_ = last_velocity_ + a_ * dt_;

    if(v_ > max_velocity_){
        v_ = max_velocity_;
    }
    else if(v_ < 0.01){
        v_ = 0.0;
    }
    last_velocity_ = v_;

    geometry_msgs::msg::Twist twist_msg;

    twist_msg.linear.x = v_;
    twist_msg.angular.z = w_;

    pub_cmd_vel->publish(twist_msg);
}

void CaccController::set_standstill_distance(const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request, std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response){
    RCLCPP_INFO(this->get_logger(), "Setting standstill distance to %f", request->value);
    standstill_distance_ = request->value;
    response->success = true;
}

void CaccController::set_time_gap(const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request, std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response){
    RCLCPP_INFO(this->get_logger(), "Setting time gap to %f", request->value);
    time_gap_ = request->value;
    response->success = true;
}

void CaccController::set_cacc_enable(const std::shared_ptr<auna_msgs::srv::SetBool::Request> request, std::shared_ptr<auna_msgs::srv::SetBool::Response> response){
    if(request->value){
        RCLCPP_INFO(this->get_logger(), "Enabling CACC controller");
        timer_->reset();
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Disabling CACC controller");
        timer_->cancel();
    }
    response->success = true;
}

//service server callback for auto_mode
void CaccController::set_auto_mode(const std::shared_ptr<auna_msgs::srv::SetBool::Request> request, std::shared_ptr<auna_msgs::srv::SetBool::Response> response){
    if(request->value){
        if(auto_mode_ready_){
            RCLCPP_INFO(this->get_logger(), "Enabling auto mode");
            auto_mode_ = true;
            timer_->reset();
            setup_timer_->cancel();
        }
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Disabling auto mode");

        geometry_msgs::msg::Twist twist_msg;

        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;

        pub_cmd_vel->publish(twist_msg);

        auto_mode_ = false;
        // if all last messages exist, start timer
        if (last_cam_msg_ == nullptr || last_odom_msg_ == nullptr || last_pose_msg_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for messages to start CACC controller");
            setup_timer_->reset();
            timer_->cancel();
        }
    }
    response->success = true;

}

