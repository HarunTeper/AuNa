#include "auna_omnet/omnet_transmitter.hpp"

// Create subscribers to get robot data and publisher to send to Artery and OMNeT++
OmnetTransmitter::OmnetTransmitter(std::string robot_name):Node("omnet_transmitter_node")
{
    pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("global_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){pose_callback(msg);});
    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg){odom_callback(msg);});

    timer = this->create_wall_timer(std::chrono::milliseconds(), [this](){cam_callback();});
    publisher = this->create_publisher<etsi_its_msgs::msg::CAM>("cam_out", 2);

    this->robot_name_ = robot_name;
}

// Send CAM data to Artery and OMNeT++ with most recently received data
void OmnetTransmitter::cam_callback()
{
    etsi_its_msgs::msg::CAM msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = robot_name_;
    
    msg.its_header.message_id = msg.its_header.MESSAGE_ID_CAM;
    msg.station_type.value = msg.station_type.PASSENGER_CAR;

    // Scale car and adjust for data format
    msg.reference_position.longitude = this->longitude_*scale_factor_*10; //0.1m
    msg.reference_position.latitude = this->latitude_*scale_factor_*10; //0.1m
    msg.reference_position.altitude.value = this->altitude_*scale_factor_*100; //0.01m

    // Adjust for data format
    msg.high_frequency_container.heading.value = this->heading_*10;//0.1 degree
    // Scale car and adjust for data format
    msg.high_frequency_container.speed.value = this->speed_*scale_factor_*100;//0.01 m/s
    msg.high_frequency_container.drive_direction.value = this->speed_<0;
    // Adjust for data format
    msg.high_frequency_container.vehicle_length.value = 0.49*scale_factor_;//m
    msg.high_frequency_container.vehicle_width.value = 0.18*scale_factor_;//m
    // Scale car and adjust for data format
    msg.high_frequency_container.longitudinal_acceleration.value = this->acceleration_*scale_factor_*10;//0.1m/s^2
    // Scale car data format
    msg.high_frequency_container.curvature.value = this->curvature_/scale_factor_;//1/m
    // Adjust for data format
    msg.high_frequency_container.yaw_rate.value = this->yaw_rate_*100;//0.01degree/s

    publisher->publish(msg);
}

// Receive and save the odometry data for speed, acceleration, yaw rate and curvature
void OmnetTransmitter::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    float old_speed = this->speed_;
    this->speed_ = sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2));//absolute speed without direction
    this->acceleration_ = (this->speed_-old_speed)/(1000/publish_period_);
    this->yaw_rate_ = msg->twist.twist.angular.z; //yaw rate in radians/s
    this->yaw_rate_ = this->yaw_rate_ * 180 / M_PI; //yaw rate in degree/s
    this->curvature_ = this->yaw_rate_/std::max(0.01f,this->speed_);
}

// Receive and save the robot pose data
void OmnetTransmitter::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->longitude_ = msg->pose.position.x;
    this->latitude_ = msg->pose.position.y;
    this->altitude_ = msg->pose.position.z;

    // Determine yaw from orientation quaternion
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

    this->heading_ = yaw; // get heading in radians
    this->heading_ = this->heading_ * 180 / M_PI; // get heading in degree
    this->heading_ = this->heading_ - 360 * floor( this->heading_ / 360 ); // get heading in interval [0,360]
}