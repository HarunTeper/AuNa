#include "auna_comm/cam_communication.hpp"

#include <etsi_its_msgs_utils/cam_access.hpp>

#include <etsi_its_cam_msgs/msg/cam.hpp>

#include <cmath>

CamCommunication::CamCommunication() : Node("cam_communication")
{
  // Declare and get parameters with input validation
  this->declare_parameter("filter_index", 0);
  this->declare_parameter("robot_index", 0);
  this->declare_parameter("vehicle_length", 0.4);
  this->declare_parameter("vehicle_width", 0.2);

  filter_index_ = this->get_parameter("filter_index").as_int();
  robot_index_ = this->get_parameter("robot_index").as_int();
  vehicle_length_ = this->get_parameter("vehicle_length").as_double();
  vehicle_width_ = this->get_parameter("vehicle_width").as_double();

  // Validate vehicle dimensions
  if (vehicle_length_ <= 0.0 || vehicle_width_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid vehicle dimensions specified");
    throw std::invalid_argument("Vehicle dimensions must be positive");
  }

  // Log initialization with essential information
  RCLCPP_INFO(
    this->get_logger(), "Initializing CAM Communication - Robot%d %s", robot_index_,
    robot_index_ == 0 ? "(leader)" : "(follower)");

  // Configuration warnings
  if (robot_index_ == 0 && filter_index_ != -1) {
    RCLCPP_WARN(
      this->get_logger(), "Robot0 (leader) should not process incoming CAMs (filter_index=%d)",
      filter_index_);
  } else if (robot_index_ > 0 && filter_index_ != (robot_index_ - 1)) {
    RCLCPP_WARN(
      this->get_logger(), "Robot%d should receive from Robot%d, not Robot%d", robot_index_,
      robot_index_ - 1, filter_index_);
  }

  // QoS settings for CAM messages
  rclcpp::QoS qos_cam(2);
  qos_cam.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos_cam.durability(rclcpp::DurabilityPolicy::TransientLocal);
  qos_cam.history(rclcpp::HistoryPolicy::KeepLast);

  // Publishers
  cam_publisher_ = this->create_publisher<etsi_its_cam_msgs::msg::CAM>("/cam", qos_cam);

  // Subscribers
  cam_subscriber_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
    "/cam", qos_cam,
    [this](etsi_its_cam_msgs::msg::CAM::SharedPtr msg) { this->cam_callback(msg); });

  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "global_pose", 2,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->pose_callback(msg); });

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->odom_callback(msg); });

  // Timer for checking CAM generation conditions
  timer_ = this->create_wall_timer(T_CheckCamGen, [this]() { this->timer_callback(); });

  // Initialize timing
  last_cam_msg_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "CAM Communication initialized");
  RCLCPP_DEBUG(
    this->get_logger(),
    "CAM generation parameters: T_GenCamMin=%ld ms, T_GenCamMax=%ld ms, T_CheckCamGen=%ld ms",
    T_GenCamMin.count(), T_GenCamMax.count(), T_CheckCamGen.count());
}

void CamCommunication::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
{
  int received_station_id = static_cast<int>(etsi_its_cam_msgs::access::getStationID(msg->header));
  RCLCPP_DEBUG(  // Changed from INFO to DEBUG
    this->get_logger(), "Received CAM from Robot%d (my index=%d, following Robot%d)",
    received_station_id, this->robot_index_, this->filter_index_);

  if (received_station_id == this->filter_index_) {
    RCLCPP_DEBUG(  // Changed from INFO to DEBUG
      this->get_logger(), "Processing CAM from Robot%d", this->filter_index_);
  }
}

void CamCommunication::timer_callback()
{
  auto now = this->now();
  rclcpp::Duration elapsed_time = now - last_cam_msg_time_;

  // Check if minimum time has elapsed (T_GenCam_DCC)
  if (elapsed_time < T_GenCamMin) {
    return;
  }

  // Condition 1: Check dynamics (heading, position, speed changes)
  bool dynamics_trigger = false;

  // 4 degrees heading change
  double heading_diff = std::abs(this->heading_ - last_cam_msg_heading_) * 180.0 / M_PI;
  if (heading_diff > 4.0) dynamics_trigger = true;

  // 4 meters position change
  double position_diff = std::sqrt(
    std::pow(this->longitude_ - last_cam_msg_longitude_, 2) +
    std::pow(this->latitude_ - last_cam_msg_latitude_, 2));
  if (position_diff > 4.0) dynamics_trigger = true;

  // 0.5 m/s speed change
  double speed_diff = std::abs(this->speed_ - last_cam_msg_speed_);
  if (speed_diff > 0.5) dynamics_trigger = true;

  // Condition 2: Maximum time exceeded
  bool timeout_trigger = elapsed_time >= T_GenCamMax;

  if (dynamics_trigger || timeout_trigger) {
    publish_cam_msg(dynamics_trigger ? "dynamics" : "timeout");
  }
}

void CamCommunication::publish_cam_msg(const std::string & trigger)
{
  RCLCPP_DEBUG(this->get_logger(), "Publishing CAM (trigger: %s)", trigger.c_str());

  etsi_its_cam_msgs::msg::CAM msg;

  // ITS PDU Header (Section 7.2)
  msg.header.protocol_version = 2;  // Per spec
  msg.header.message_id = etsi_its_cam_msgs::msg::ItsPduHeader::MESSAGE_ID_CAM;
  msg.header.station_id.value = this->robot_index_;

  // Generation Delta Time (Section B.3)
  // TimestampIts represents time in milliseconds since 2004-01-01T00:00:00.000Z
  // Value wrapped to 65536
  uint16_t gen_delta_time =
    (this->now().nanoseconds() / 1000000) % 65536;  // Milliseconds mod 65536
  msg.cam.generation_delta_time.value = gen_delta_time;

  // Basic Container (Section 7.3)
  auto & basic = msg.cam.cam_parameters.basic_container;
  basic.station_type.value = etsi_its_cam_msgs::msg::StationType::PASSENGER_CAR;

  // Reference Position (Section B.19)
  // Latitude: +/-90°, 1/10 microdegree precision
  basic.reference_position.latitude.value = static_cast<int32_t>(this->latitude_ * 10000000);
  // Longitude: +/-180°, 1/10 microdegree precision
  basic.reference_position.longitude.value = static_cast<int32_t>(this->longitude_ * 10000000);
  // Altitude: -1000m to +6000m, 0.01m precision
  basic.reference_position.altitude.altitude_value.value =
    static_cast<int32_t>(this->altitude_ * 100);

  // Position Confidence (Section B.19)
  basic.reference_position.position_confidence_ellipse.semi_major_confidence.value =
    etsi_its_cam_msgs::msg::SemiAxisLength::UNAVAILABLE;
  basic.reference_position.position_confidence_ellipse.semi_minor_confidence.value =
    etsi_its_cam_msgs::msg::SemiAxisLength::UNAVAILABLE;
  basic.reference_position.position_confidence_ellipse.semi_major_orientation.value =
    etsi_its_cam_msgs::msg::HeadingValue::UNAVAILABLE;
  basic.reference_position.altitude.altitude_confidence.value =
    etsi_its_cam_msgs::msg::AltitudeConfidence::UNAVAILABLE;

  // High Frequency Container (Section 7.4)
  auto & hf = msg.cam.cam_parameters.high_frequency_container;
  hf.choice =
    etsi_its_cam_msgs::msg::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;

  auto & vhf = hf.basic_vehicle_container_high_frequency;

  // Heading (Section B.21)
  // WGS84 north (0°), east (90°), south (180°), west (270°). 0.1° step
  vhf.heading.heading_value.value = static_cast<uint16_t>(this->heading_ * 180.0 / M_PI * 10);
  vhf.heading.heading_confidence.value = etsi_its_cam_msgs::msg::HeadingConfidence::UNAVAILABLE;

  // Speed (Section B.22)
  // 0.01 m/s precision
  vhf.speed.speed_value.value = static_cast<uint16_t>(std::abs(this->speed_) * 100);
  vhf.speed.speed_confidence.value = etsi_its_cam_msgs::msg::SpeedConfidence::UNAVAILABLE;

  // Drive Direction (Section B.25)
  if (this->speed_ > 0) {
    vhf.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::FORWARD;
  } else if (this->speed_ < 0) {
    vhf.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::BACKWARD;
  } else {
    vhf.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::UNAVAILABLE;
  }

  // Vehicle Length and Width (Section B.35, B.36)
  // Length: 0.1 meter precision
  vhf.vehicle_length.vehicle_length_value.value = static_cast<uint16_t>(this->vehicle_length_ * 10);
  vhf.vehicle_length.vehicle_length_confidence_indication.value =
    etsi_its_cam_msgs::msg::VehicleLengthConfidenceIndication::UNAVAILABLE;
  // Width: 0.1 meter precision
  vhf.vehicle_width.value = static_cast<uint16_t>(this->vehicle_width_ * 10);

  // Longitudinal Acceleration (Section B.26)
  // 0.1 m/s² precision
  vhf.longitudinal_acceleration.longitudinal_acceleration_value.value =
    static_cast<int16_t>(this->acceleration_ * 10);
  vhf.longitudinal_acceleration.longitudinal_acceleration_confidence.value =
    etsi_its_cam_msgs::msg::AccelerationConfidence::UNAVAILABLE;

  // Curvature (Section B.31)
  // 1/10000 per meter precision
  if (std::isnan(this->curvature_)) {
    vhf.curvature.curvature_value.value = etsi_its_cam_msgs::msg::CurvatureValue::UNAVAILABLE;
  } else {
    vhf.curvature.curvature_value.value = static_cast<int16_t>(this->curvature_ * 10000);
  }
  vhf.curvature.curvature_confidence.value =
    etsi_its_cam_msgs::msg::CurvatureConfidence::UNAVAILABLE;

  // Curvature Calculation Mode (Section B.32)
  vhf.curvature_calculation_mode.value =
    etsi_its_cam_msgs::msg::CurvatureCalculationMode::YAW_RATE_USED;

  // Yaw Rate (Section B.33)
  // 0.01 degrees per second precision
  if (std::isnan(this->yaw_rate_)) {
    vhf.yaw_rate.yaw_rate_value.value = etsi_its_cam_msgs::msg::YawRateValue::UNAVAILABLE;
  } else {
    vhf.yaw_rate.yaw_rate_value.value = static_cast<int16_t>(this->yaw_rate_ * 180.0 / M_PI * 100);
  }
  vhf.yaw_rate.yaw_rate_confidence.value = etsi_its_cam_msgs::msg::YawRateConfidence::UNAVAILABLE;

  // Optional: Low Frequency Container (Section 7.4)
  // Not including in this implementation as it's optional
  // Would contain vehicle role, exterior lights, and path history

  cam_publisher_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "CAM published");

  // Update last message information for trigger condition checking
  last_cam_msg_time_ = this->now();
  last_cam_msg_speed_ = this->speed_;
  last_cam_msg_latitude_ = this->latitude_;
  last_cam_msg_longitude_ = this->longitude_;
  last_cam_msg_heading_ = this->heading_;
}

void CamCommunication::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received odometry update");
  this->speed_ = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));

  if (last_odom_msg_ != nullptr) {
    this->old_speed_ = sqrt(
      pow(last_odom_msg_->twist.twist.linear.x, 2) +
      pow(last_odom_msg_->twist.twist.linear.y, 2));  // absolute speed without direction
    this->acceleration_ =
      (this->speed_ - this->old_speed_) /
      (msg->header.stamp.sec - last_odom_msg_->header.stamp.sec +
       (msg->header.stamp.nanosec - last_odom_msg_->header.stamp.nanosec) / 1000000000.0);
  } else {
    this->acceleration_ = 0;
  }
  this->yaw_rate_ = msg->twist.twist.angular.z;

  if (this->speed_ == 0) {
    this->curvature_ = 0;
  } else {
    this->curvature_ = this->yaw_rate_ / this->speed_;
  }

  if (msg->twist.twist.linear.x > 0) {
    this->drive_direction_ = 1;
  } else if (msg->twist.twist.linear.x < 0) {
    this->drive_direction_ = -1;
  } else {
    this->drive_direction_ = 0;
  }

  this->last_odom_msg_ = msg;
  RCLCPP_DEBUG(
    this->get_logger(),
    "Updated vehicle dynamics - Speed: %.2f m/s, Acc: %.2f m/s², Yaw rate: %.2f rad/s",
    this->speed_, this->acceleration_, this->yaw_rate_);
}

void CamCommunication::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received pose update");
  this->longitude_ = msg->pose.position.x;
  this->latitude_ = msg->pose.position.y;
  this->altitude_ = msg->pose.position.z;

  tf2::Quaternion quat(
    msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
    msg->pose.orientation.w);
  quat.normalize();
  tf2::Matrix3x3 matrix(quat);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  matrix.getRPY(roll, pitch, yaw);

  this->heading_ = yaw;
  RCLCPP_DEBUG(
    this->get_logger(), "Updated vehicle pose - Pos: (%.6f, %.6f, %.2f), Heading: %.2f deg",
    this->longitude_, this->latitude_, this->altitude_, this->heading_ * 180.0 / M_PI);
}