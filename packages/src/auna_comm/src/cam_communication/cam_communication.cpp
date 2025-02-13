#include "auna_comm/cam_communication.hpp"

#include <etsi_its_msgs_utils/cam_access.hpp>

#include <etsi_its_cam_msgs/msg/cam.hpp>

#include <cmath>

CamCommunication::CamCommunication() : Node("cam_communication")
{
  this->declare_parameter("filter_index", 0);
  this->declare_parameter("robot_index", 0);
  this->declare_parameter("vehicle_length", 0.4);
  this->declare_parameter("vehicle_width", 0.2);

  this->filter_index_ = this->get_parameter("filter_index").as_int();
  this->robot_index_ = this->get_parameter("robot_index").as_int();
  this->vehicle_length_ = this->get_parameter("vehicle_length").as_double();
  this->vehicle_width_ = this->get_parameter("vehicle_width").as_double();

  // Enhanced parameter logging and validation for chain following
  RCLCPP_INFO(this->get_logger(), "Initializing CAM Communication node with parameters:");
  RCLCPP_INFO(
    this->get_logger(), "  - robot_index: %d (This node will send CAMs with stationID=%d)",
    this->robot_index_, this->robot_index_);
  RCLCPP_INFO(
    this->get_logger(), "  - filter_index: %d (This node will process CAMs from stationID=%d)",
    this->filter_index_, this->filter_index_);
  RCLCPP_INFO(this->get_logger(), "  - vehicle_length: %.2f", this->vehicle_length_);
  RCLCPP_INFO(this->get_logger(), "  - vehicle_width: %.2f", this->vehicle_width_);

  // Validate chain following configuration
  if (this->robot_index_ == 0 && this->filter_index_ != -1) {
    RCLCPP_WARN(
      this->get_logger(),
      "Robot0 (leader) is configured to receive CAMs (filter_index=%d). "
      "The leader should not process incoming CAMs.",
      this->filter_index_);
  } else if (this->robot_index_ > 0 && this->filter_index_ != (this->robot_index_ - 1)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Robot%d (follower) is configured to receive CAMs from stationID=%d. "
      "In chain following, it should receive from Robot%d (stationID=%d).",
      this->robot_index_, this->filter_index_, this->robot_index_ - 1, this->robot_index_ - 1);
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Chain following configuration correct: Robot%d %s", this->robot_index_,
      this->robot_index_ == 0
        ? "(leader)"
        : std::string("following Robot" + std::to_string(this->robot_index_ - 1)).c_str());
  }

  cam_publisher_ = this->create_publisher<etsi_its_cam_msgs::msg::CAM>("/cam", 2);
  cam_subscriber_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
    "/cam", 2,
    [this](etsi_its_cam_msgs::msg::CAM::SharedPtr msg) -> void { this->cam_callback(msg); });
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() -> void { this->timer_callback(); });

  this->pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "global_pose", 2,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_callback(msg); });
  this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

  last_cam_msg_time_ = this->now();  // Initialize for time comparisons
}

void CamCommunication::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
{
  int received_station_id = static_cast<int>(etsi_its_cam_msgs::access::getStationID(msg->header));
  RCLCPP_INFO(
    this->get_logger(), "Received CAM message from Robot%d (my index=%d, following Robot%d)",
    received_station_id, this->robot_index_, this->filter_index_);

  if (received_station_id == this->filter_index_) {
    RCLCPP_INFO(
      this->get_logger(), "Processing CAM message from matching filter_index: %d",
      this->filter_index_);
  }
}

void CamCommunication::timer_callback()
{
  rclcpp::Duration elapsed_time = this->now() - last_cam_msg_time_;
  RCLCPP_DEBUG(
    this->get_logger(), "Timer callback - Time since last CAM: %.2f seconds",
    elapsed_time.seconds());

  if (elapsed_time >= std::chrono::milliseconds(1000)) {
    RCLCPP_INFO(this->get_logger(), "Publishing CAM due to timeout (>1s since last message)");
    publish_cam_msg("timeout");
  } else {
    double speed_diff = fabs(this->speed_ - last_cam_msg_speed_);
    double position_diff = sqrt(
      pow(this->longitude_ - last_cam_msg_longitude_, 2) +
      pow(this->latitude_ - last_cam_msg_latitude_, 2));
    double heading_diff = fabs(this->heading_ - last_cam_msg_heading_) * 180.0 / M_PI;

    RCLCPP_DEBUG(this->get_logger(), "Checking trigger conditions:");
    RCLCPP_DEBUG(this->get_logger(), "  Speed diff: %.3f (threshold: 0.05)", speed_diff);
    RCLCPP_DEBUG(this->get_logger(), "  Position diff: %.3f (threshold: 0.4)", position_diff);
    RCLCPP_DEBUG(this->get_logger(), "  Heading diff: %.3f degrees (threshold: 4.0)", heading_diff);

    if (speed_diff > 0.05) {
      RCLCPP_INFO(
        this->get_logger(), "Publishing CAM due to speed change (%.2f -> %.2f m/s)",
        last_cam_msg_speed_, this->speed_);
      publish_cam_msg("speed");
    } else if (position_diff > 0.4) {
      RCLCPP_INFO(
        this->get_logger(), "Publishing CAM due to position change (%.2f m)", position_diff);
      publish_cam_msg("position");
    } else if (heading_diff > 4.0) {
      RCLCPP_INFO(
        this->get_logger(), "Publishing CAM due to heading change (%.2f degrees)", heading_diff);
      publish_cam_msg("heading");
    }
  }
}

void CamCommunication::publish_cam_msg(std::string frame_id)
{
  RCLCPP_INFO(this->get_logger(), "Publishing CAM message (trigger: %s)", frame_id.c_str());
  RCLCPP_DEBUG(this->get_logger(), "Vehicle state:");
  RCLCPP_DEBUG(
    this->get_logger(), "  Position: (%.6f, %.6f, %.2f)", this->latitude_, this->longitude_,
    this->altitude_);
  RCLCPP_DEBUG(this->get_logger(), "  Speed: %.2f m/s", this->speed_);
  RCLCPP_DEBUG(this->get_logger(), "  Heading: %.2f deg", this->heading_ * 180.0 / M_PI);
  RCLCPP_DEBUG(this->get_logger(), "  Acceleration: %.2f m/s²", this->acceleration_);
  RCLCPP_DEBUG(this->get_logger(), "  Yaw rate: %.2f rad/s", this->yaw_rate_);
  RCLCPP_DEBUG(this->get_logger(), "  Curvature: %.4f", this->curvature_);
  RCLCPP_DEBUG(this->get_logger(), "  Drive direction: %.0f", this->drive_direction_);

  etsi_its_cam_msgs::msg::CAM msg;

  // --- ItsPduHeader ---
  etsi_its_cam_msgs::access::setItsPduHeader(
    msg.header, etsi_its_cam_msgs::msg::ItsPduHeader::MESSAGE_ID_CAM, this->robot_index_);

  // --- CoopAwareness ---
  etsi_its_cam_msgs::access::setGenerationDeltaTime(
    msg, this->now().nanoseconds(),
    etsi_its_msgs::getLeapSecondInsertionsSince2004(this->now().seconds()));

  // --- CamParameters -> BasicContainer ---
  etsi_its_cam_msgs::access::setStationType(
    msg, etsi_its_cam_msgs::msg::StationType::PASSENGER_CAR);
  etsi_its_cam_msgs::access::setReferencePosition(
    msg, this->latitude_, this->longitude_, this->altitude_);

  // --- CamParameters -> HighFrequencyContainer ---
  etsi_its_cam_msgs::msg::BasicVehicleContainerHighFrequency high_freq_container;

  // Heading
  etsi_its_cam_msgs::access::setHeading(high_freq_container.heading, this->heading_ * 180.0 / M_PI);
  high_freq_container.heading.heading_confidence.value =
    etsi_its_cam_msgs::msg::HeadingConfidence::UNAVAILABLE;
  // Speed
  etsi_its_cam_msgs::access::setSpeed(high_freq_container.speed, this->speed_);
  high_freq_container.speed.speed_confidence.value =
    etsi_its_cam_msgs::msg::SpeedConfidence::UNAVAILABLE;

  // Drive Direction
  if (this->drive_direction_ == 1) {
    high_freq_container.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::FORWARD;
  } else if (this->drive_direction_ == -1) {
    high_freq_container.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::BACKWARD;
  } else {
    high_freq_container.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::UNAVAILABLE;
  }

  // Vehicle Length and Width (in 10 cm units)
  etsi_its_cam_msgs::access::setVehicleLength(
    high_freq_container.vehicle_length, this->vehicle_length_);
  high_freq_container.vehicle_length.vehicle_length_confidence_indication.value =
    etsi_its_cam_msgs::msg::VehicleLengthConfidenceIndication::UNAVAILABLE;
  etsi_its_cam_msgs::access::setVehicleWidth(
    high_freq_container.vehicle_width, this->vehicle_width_);

  // Longitudinal Acceleration (in 0.1 m/s^2)
  etsi_its_cam_msgs::access::setLongitudinalAcceleration(
    high_freq_container.longitudinal_acceleration, this->acceleration_);
  high_freq_container.longitudinal_acceleration.longitudinal_acceleration_confidence.value =
    etsi_its_cam_msgs::msg::AccelerationConfidence::UNAVAILABLE;

  // Curvature  and CurvatureCalculationMode, no access function in etsi_its_cam_msgs::access
  high_freq_container.curvature.curvature_value.value =
    static_cast<int16_t>(std::round(this->curvature_ * 10000.0));  // Scale by 10000
  high_freq_container.curvature.curvature_confidence.value =
    etsi_its_cam_msgs::msg::CurvatureConfidence::UNAVAILABLE;
  high_freq_container.curvature_calculation_mode.value =
    etsi_its_cam_msgs::msg::CurvatureCalculationMode::YAW_RATE_USED;

  // Yaw Rate no access function in etsi_its_cam_msgs::access
  // Convert rad/s to 0.01 degrees/s, and handle the special UNAVAILABLE case.
  constexpr int16_t UNAVAILABLE_YAW_RATE = 32767;
  if (std::isnan(this->yaw_rate_)) {
    high_freq_container.yaw_rate.yaw_rate_value.value = UNAVAILABLE_YAW_RATE;
  } else {
    high_freq_container.yaw_rate.yaw_rate_value.value = static_cast<int16_t>(
      std::round(this->yaw_rate_ * 180.0 / M_PI * 100.0));  // rad/s to 0.01 deg/s
  }

  high_freq_container.yaw_rate.yaw_rate_confidence.value =
    etsi_its_cam_msgs::msg::YawRateConfidence::UNAVAILABLE;  // Set confidence

  // Set the CHOICE in the HighFrequencyContainer *after* populating the struct
  msg.cam.cam_parameters.high_frequency_container.choice =
    etsi_its_cam_msgs::msg::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
  msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency =
    high_freq_container;  // Assign the populated struct

  cam_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "CAM message published successfully");

  // --- Update last message information for delta calculations ---
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