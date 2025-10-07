// Copyright 2025 Harun Teper
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "auna_comm/cam_communication.hpp"

#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <etsi_its_msgs_utils/cam_access.hpp>

CamCommunication::CamCommunication()
: Node("cam_communication")
{
  // Declare and get parameters with input validation
  this->declare_parameter("filter_index", 0);
  this->declare_parameter("robot_index", 0);
  this->declare_parameter("vehicle_length", 0.4);
  this->declare_parameter("vehicle_width", 0.2);

  // Add parameters for CAM message logging
  this->declare_parameter("enable_cam_logging", false);
  this->declare_parameter(
    "cam_log_file_path",
    "/home/vscode/workspace/cam_messages.log");

  filter_index_ = this->get_parameter("filter_index").as_int();
  robot_index_ = this->get_parameter("robot_index").as_int();
  vehicle_length_ = this->get_parameter("vehicle_length").as_double();
  vehicle_width_ = this->get_parameter("vehicle_width").as_double();

  // Initialize CAM logging
  enable_cam_logging_ = this->get_parameter("enable_cam_logging").as_bool();
  cam_log_file_path_ = this->get_parameter("cam_log_file_path").as_string();

  if (enable_cam_logging_) {
    // Create a unique log file for each robot if multiple robots are present
    if (robot_index_ > 0) {
      // Insert robot index before file extension
      size_t dot_pos = cam_log_file_path_.find_last_of('.');
      if (dot_pos != std::string::npos) {
        cam_log_file_path_.insert(
          dot_pos,
          "_robot" + std::to_string(robot_index_));
      } else {
        cam_log_file_path_ += "_robot" + std::to_string(robot_index_);
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "CAM message logging enabled. Writing to: %s",
      cam_log_file_path_.c_str());
    cam_log_file_.open(cam_log_file_path_, std::ios::out | std::ios::trunc);

    if (cam_log_file_.is_open()) {
      cam_log_file_ << "=== CAM Message Log for Robot " << robot_index_ << " ("
                    << (robot_index_ == 0 ? "LEADER" : "FOLLOWER")
                    << ") ===" << std::endl;
      cam_log_file_ << "Timestamp,Action,StationID,GenDeltaTime,Longitude,"
        "Latitude,Heading,Speed,"
        "Acceleration,YawRate,Curvature,Details"
                    << std::endl;
      cam_log_file_.flush();
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to open CAM log file: %s",
        cam_log_file_path_.c_str());
      enable_cam_logging_ = false;
    }
  }

  // Validate vehicle dimensions
  if (vehicle_length_ <= 0.0 || vehicle_width_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid vehicle dimensions specified");
    throw std::invalid_argument("Vehicle dimensions must be positive");
  }

  // Log initialization with essential information
  RCLCPP_INFO(
    this->get_logger(),
    "[Robot%d] Initializing CAM Communication - %s", robot_index_,
    robot_index_ == 0 ? "LEADER" : "FOLLOWER");

  // Configuration warnings
  if (robot_index_ == 0 && filter_index_ != -1) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Robot%d] Leader should not process incoming CAMs (filter_index=%d)",
      robot_index_, filter_index_);
  } else if (robot_index_ > 0 && filter_index_ != (robot_index_ - 1)) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Robot%d] Should receive from Robot%d, not Robot%d",
      robot_index_, robot_index_ - 1, filter_index_);
  }

  // QoS settings for CAM messages
  rclcpp::QoS qos_cam(2);
  qos_cam.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos_cam.durability(rclcpp::DurabilityPolicy::TransientLocal);
  qos_cam.history(rclcpp::HistoryPolicy::KeepLast);

  // Publishers
  cam_publisher_ =
    this->create_publisher<etsi_its_cam_msgs::msg::CAM>("/cam", qos_cam);

  // Subscribers
  cam_subscriber_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
    "/cam", qos_cam, [this](etsi_its_cam_msgs::msg::CAM::SharedPtr msg) {
      this->cam_callback(msg);
    });

  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "global_pose", 2,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      this->pose_callback(msg);
    });

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered", 2,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      this->odom_callback(msg);
    });

  // Timer for checking CAM generation conditions
  timer_ = this->create_wall_timer(
    T_CheckCamGen,
    [this]() {this->timer_callback();});

  // Initialize timing
  last_cam_msg_time_ = this->now();

  RCLCPP_INFO(
    this->get_logger(), "[Robot%d] CAM Communication initialized",
    robot_index_);
  RCLCPP_DEBUG(
    this->get_logger(),
    "[Robot%d] CAM generation parameters: T_GenCamMin=%ld ms, "
    "T_GenCamMax=%ld ms, "
    "T_CheckCamGen=%ld ms",
    robot_index_, T_GenCamMin.count(), T_GenCamMax.count(),
    T_CheckCamGen.count());

  // Add cleanup for file logging
  rclcpp::on_shutdown(
    [this]() {
      if (enable_cam_logging_ && cam_log_file_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Closing CAM log file");
        cam_log_file_ << "=== Log closed at " << std::fixed
                      << std::setprecision(3) << this->now().seconds()
                      << " ===" << std::endl;
        cam_log_file_.close();
      }
    });
}

void CamCommunication::cam_callback(
  const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
{
  int received_station_id =
    static_cast<int>(etsi_its_cam_msgs::access::getStationID(msg->header));

  // Log all received CAMs at INFO level with clear source identification
  RCLCPP_DEBUG(
    this->get_logger(),
    "[Robot%d] RX CAM from Robot%d | Speed: %.2f m/s | Gen time: %u | %s",
    robot_index_, received_station_id,
    msg->cam.cam_parameters.high_frequency_container
    .basic_vehicle_container_high_frequency.speed.speed_value.value /
    100.0,
    msg->cam.generation_delta_time.value,
    received_station_id == this->filter_index_ ? "PROCESSING" : "IGNORING");

  // Log to file if enabled
  if (enable_cam_logging_ && cam_log_file_.is_open()) {
    auto now = this->now();
    double timestamp = now.seconds() + now.nanoseconds() * 1e-9;

    // Get speed value in m/s
    double speed_value =
      msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.speed.speed_value.value /
      100.0;

    // Get position values
    double longitude = msg->cam.cam_parameters.basic_container
      .reference_position.longitude.value /
      10000000.0;
    double latitude = msg->cam.cam_parameters.basic_container.reference_position
      .latitude.value /
      10000000.0;

    // Get heading value
    uint16_t raw_heading =
      msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.heading.heading_value.value;
    double heading_degrees =
      (raw_heading == etsi_its_cam_msgs::msg::HeadingValue::UNAVAILABLE) ?
      0.0 :
      (raw_heading % 3600) / 10.0;

    // Get yaw rate and determine if available
    int16_t raw_yaw_rate = msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.yaw_rate
      .yaw_rate_value.value;
    std::string yaw_rate_str =
      (raw_yaw_rate == etsi_its_cam_msgs::msg::YawRateValue::UNAVAILABLE) ?
      "UNAVAILABLE" :
      std::to_string(raw_yaw_rate / 100.0);

    // Get curvature and determine if available
    int16_t raw_curvature = msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency
      .curvature.curvature_value.value;
    std::string curvature_str =
      (raw_curvature == etsi_its_cam_msgs::msg::CurvatureValue::UNAVAILABLE) ?
      "UNAVAILABLE" :
      std::to_string(raw_curvature / 10000.0);

    // Additional details to include
    std::string processing = (received_station_id == this->filter_index_) ?
      "PROCESSING" :
      "IGNORING";

    // Write to log file (CSV format for easier analysis)
    cam_log_file_ << std::fixed << std::setprecision(6) << timestamp << ","
                  << "RX," << received_station_id << ","
                  << msg->cam.generation_delta_time.value << "," << longitude
                  << "," << latitude << "," << heading_degrees << ","
                  << speed_value << ","
                  << "N/A"
                  << ","  // Acceleration not directly in CAM
                  << yaw_rate_str << "," << curvature_str << "," << processing
                  << std::endl;
  }

  if (received_station_id == this->filter_index_) {
    // Get heading value and properly handle potential out-of-range values
    uint16_t raw_heading =
      msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.heading.heading_value.value;

    double heading_degrees;
    if (raw_heading == etsi_its_cam_msgs::msg::HeadingValue::UNAVAILABLE) {
      heading_degrees = 0.0;  // Default when unavailable
    } else {
      // Ensure the heading is in valid range (0-3600) before converting to
      // degrees
      raw_heading = raw_heading % 3600;
      heading_degrees = raw_heading / 10.0;
    }

    // Additional debug information about received message
    RCLCPP_DEBUG(
      this->get_logger(),
      "[Robot%d] CAM details | Position: (%.6f, %.6f) | Heading: "
      "%.2f° (raw: %u) | Yaw rate: "
      "%.2f°/s",
      robot_index_,
      msg->cam.cam_parameters.basic_container.reference_position
      .longitude.value /
      10000000.0,
      msg->cam.cam_parameters.basic_container.reference_position
      .latitude.value /
      10000000.0,
      heading_degrees, raw_heading,
      msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.yaw_rate
      .yaw_rate_value.value /
      100.0);
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
  double heading_diff =
    std::abs(this->heading_ - last_cam_msg_heading_) * 180.0 / M_PI;
  if (heading_diff > 4.0) {
    dynamics_trigger = true;
  }

  // 4 meters position change
  double position_diff =
    std::sqrt(
    std::pow(this->longitude_ - last_cam_msg_longitude_, 2) +
    std::pow(this->latitude_ - last_cam_msg_latitude_, 2));
  if (position_diff > 4.0) {
    dynamics_trigger = true;
  }

  // 0.5 m/s speed change
  double speed_diff = std::abs(this->speed_ - last_cam_msg_speed_);
  if (speed_diff > 0.5) {
    dynamics_trigger = true;
  }

  // Condition 2: Maximum time exceeded
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::nanoseconds(elapsed_time.nanoseconds()));

  const std::chrono::milliseconds buffer(100);
  bool timeout_trigger = elapsed_ms >= (T_GenCamMax - buffer);

  if (dynamics_trigger || timeout_trigger) {
    publish_cam_msg(dynamics_trigger ? "dynamics" : "timeout");
  }
}

void CamCommunication::publish_cam_msg(const std::string & trigger)
{
  RCLCPP_DEBUG(
    this->get_logger(), "[Robot%d] Publishing CAM (trigger: %s)",
    robot_index_, trigger.c_str());

  etsi_its_cam_msgs::msg::CAM msg;

  // ITS PDU Header (Section 7.2)
  msg.header.protocol_version = 2;  // Per spec
  msg.header.message_id = etsi_its_cam_msgs::msg::ItsPduHeader::MESSAGE_ID_CAM;
  msg.header.station_id.value = this->robot_index_;

  // Generation Delta Time (Section B.3)
  // TimestampIts represents time in milliseconds since 2004-01-01T00:00:00.000Z
  // Value wrapped to 65536
  uint16_t gen_delta_time =
    (this->now().nanoseconds() / 1000000) % 65536;    // Milliseconds mod 65536
  msg.cam.generation_delta_time.value = gen_delta_time;

  // Log the message construction process if enabled
  if (enable_cam_logging_ && cam_log_file_.is_open()) {
    auto now = this->now();
    double timestamp = now.seconds() + now.nanoseconds() * 1e-9;

    cam_log_file_ << std::fixed << std::setprecision(6) << timestamp << ","
                  << "BUILD-START," << this->robot_index_ << ","
                  << gen_delta_time << ","
                  << "N/A"
                  << ","  // Fields being constructed
                  << "N/A"
                  << ","
                  << "N/A"
                  << ","
                  << "N/A"
                  << ","
                  << "N/A"
                  << ","
                  << "N/A"
                  << ","
                  << "N/A"
                  << ","
                  << "Trigger: " << trigger << std::endl;
  }

  // Basic Container (Section 7.3)
  auto & basic = msg.cam.cam_parameters.basic_container;
  basic.station_type.value = etsi_its_cam_msgs::msg::StationType::PASSENGER_CAR;

  // Reference Position (Section B.19)
  // Latitude: +/-90°, 1/10 microdegree precision
  basic.reference_position.latitude.value =
    static_cast<int32_t>(this->latitude_ * 10000000);
  // Longitude: +/-180°, 1/10 microdegree precision
  basic.reference_position.longitude.value =
    static_cast<int32_t>(this->longitude_ * 10000000);
  // Altitude: -1000m to +6000m, 0.01m precision
  basic.reference_position.altitude.altitude_value.value =
    static_cast<int32_t>(this->altitude_ * 100);

  // Position Confidence (Section B.19)
  basic.reference_position.position_confidence_ellipse.semi_major_confidence
  .value = etsi_its_cam_msgs::msg::SemiAxisLength::UNAVAILABLE;
  basic.reference_position.position_confidence_ellipse.semi_minor_confidence
  .value = etsi_its_cam_msgs::msg::SemiAxisLength::UNAVAILABLE;
  basic.reference_position.position_confidence_ellipse.semi_major_orientation
  .value = etsi_its_cam_msgs::msg::HeadingValue::UNAVAILABLE;
  basic.reference_position.altitude.altitude_confidence.value =
    etsi_its_cam_msgs::msg::AltitudeConfidence::UNAVAILABLE;

  // High Frequency Container (Section 7.4)
  auto & hf = msg.cam.cam_parameters.high_frequency_container;
  hf.choice = etsi_its_cam_msgs::msg::HighFrequencyContainer::
    CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;

  auto & vhf = hf.basic_vehicle_container_high_frequency;

  // Heading (Section B.21)
  // WGS84 north (0°), east (90°), south (180°), west (270°). 0.1° step
  // Convert heading from radians to 0.1 degree units and ensure it's in the
  // valid range (0-3600)
  uint16_t heading_value = static_cast<uint16_t>(
    fmod(this->heading_ * 180.0 / M_PI * 10 + 3600, 3600));
  vhf.heading.heading_value.value = heading_value;
  vhf.heading.heading_confidence.value =
    etsi_its_cam_msgs::msg::HeadingConfidence::UNAVAILABLE;

  // Speed (Section B.22)
  // 0.01 m/s precision
  vhf.speed.speed_value.value =
    static_cast<uint16_t>(std::abs(this->speed_) * 100);
  vhf.speed.speed_confidence.value =
    etsi_its_cam_msgs::msg::SpeedConfidence::UNAVAILABLE;

  // Drive Direction (Section B.25)
  if (this->speed_ > 0) {
    vhf.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::FORWARD;
  } else if (this->speed_ < 0) {
    vhf.drive_direction.value =
      etsi_its_cam_msgs::msg::DriveDirection::BACKWARD;
  } else {
    vhf.drive_direction.value =
      etsi_its_cam_msgs::msg::DriveDirection::UNAVAILABLE;
  }

  // Vehicle Length and Width (Section B.35, B.36)
  // Length: 0.1 meter precision
  vhf.vehicle_length.vehicle_length_value.value =
    static_cast<uint16_t>(this->vehicle_length_ * 10);
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
    vhf.curvature.curvature_value.value =
      etsi_its_cam_msgs::msg::CurvatureValue::UNAVAILABLE;
  } else {
    vhf.curvature.curvature_value.value =
      static_cast<int16_t>(this->curvature_ * 10000);
  }
  vhf.curvature.curvature_confidence.value =
    etsi_its_cam_msgs::msg::CurvatureConfidence::UNAVAILABLE;

  // Curvature Calculation Mode (Section B.32)
  vhf.curvature_calculation_mode.value =
    etsi_its_cam_msgs::msg::CurvatureCalculationMode::YAW_RATE_USED;

  // Yaw Rate (Section B.33)
  // 0.01 degrees per second precision
  // Use small epsilon to account for floating point precision
  if (std::abs(this->speed_) < 0.001 || std::isnan(this->yaw_rate_)) {
    vhf.yaw_rate.yaw_rate_value.value =
      etsi_its_cam_msgs::msg::YawRateValue::UNAVAILABLE;
  } else {
    // Convert rad/s to 0.01 deg/s units (value * 100 = centi-degrees/s)
    vhf.yaw_rate.yaw_rate_value.value =
      static_cast<int16_t>(this->yaw_rate_ * (180.0 / M_PI) * 100);
  }
  vhf.yaw_rate.yaw_rate_confidence.value =
    etsi_its_cam_msgs::msg::YawRateConfidence::UNAVAILABLE;

  // Log the fully constructed message before publishing if enabled
  if (enable_cam_logging_ && cam_log_file_.is_open()) {
    auto now = this->now();
    double timestamp = now.seconds() + now.nanoseconds() * 1e-9;

    // Extract yaw rate value or mark as unavailable
    std::string yaw_rate_str = "UNAVAILABLE";
    if (vhf.yaw_rate.yaw_rate_value.value !=
      etsi_its_cam_msgs::msg::YawRateValue::UNAVAILABLE)
    {
      yaw_rate_str = std::to_string(vhf.yaw_rate.yaw_rate_value.value / 100.0);
    }

    // Extract curvature value or mark as unavailable
    std::string curvature_str = "UNAVAILABLE";
    if (vhf.curvature.curvature_value.value !=
      etsi_its_cam_msgs::msg::CurvatureValue::UNAVAILABLE)
    {
      curvature_str =
        std::to_string(vhf.curvature.curvature_value.value / 10000.0);
    }

    // Create detailed log entry of CAM message
    cam_log_file_ << std::fixed << std::setprecision(6) << timestamp << ","
                  << "TX," << this->robot_index_ << "," << gen_delta_time << ","
                  << this->longitude_ << "," << this->latitude_ << ","
                  << (heading_value / 10.0) << ","
                  << (vhf.speed.speed_value.value / 100.0) << ","
                  << (vhf.longitudinal_acceleration
    .longitudinal_acceleration_value.value /
    10.0)
                  << "," << yaw_rate_str << "," << curvature_str << ","
                  << "Drive direction: " << vhf.drive_direction.value
                  << " Speed: " << this->speed_ << std::endl;
  }

  // Publish the message
  cam_publisher_->publish(msg);

  // Log the CAM message with a standardized format at INFO level
  RCLCPP_DEBUG(
    this->get_logger(),
    "[Robot%d] TX CAM | Gen time: %u | Speed: %.2f m/s | Accel: "
    "%.2f m/s² | Yaw rate: %.2f°/s | "
    "Pos: (%.2f, %.2f)",
    robot_index_, gen_delta_time, this->speed_, this->acceleration_,
    this->yaw_rate_ * 180.0 / M_PI, this->longitude_,
    this->latitude_);

  // More detailed info at DEBUG level
  RCLCPP_DEBUG(
    this->get_logger(),
    "[Robot%d] TX CAM details | Heading: %.2f° | Curvature: %.6f | "
    "Trigger: %s",
    robot_index_, this->heading_ * 180.0 / M_PI, this->curvature_,
    trigger.c_str());

  // Update last message information for trigger condition checking
  last_cam_msg_time_ = this->now();
  last_cam_msg_speed_ = this->speed_;
  last_cam_msg_latitude_ = this->latitude_;
  last_cam_msg_longitude_ = this->longitude_;
  last_cam_msg_heading_ = this->heading_;
}

void CamCommunication::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_DEBUG(
    this->get_logger(), "[Robot%d] Received odometry update",
    robot_index_);
  this->speed_ = sqrt(
    pow(msg->twist.twist.linear.x, 2) +
    pow(msg->twist.twist.linear.y, 2));

  if (last_odom_msg_ != nullptr) {
    this->old_speed_ = sqrt(
      pow(last_odom_msg_->twist.twist.linear.x, 2) +
      pow(
        last_odom_msg_->twist.twist.linear.y,
        2));                          // absolute speed without direction

    double dt =
      msg->header.stamp.sec - last_odom_msg_->header.stamp.sec +
      (msg->header.stamp.nanosec - last_odom_msg_->header.stamp.nanosec) /
      1e9;

    if (dt < 0.001) {
      // Prevent division by very small numbers
      this->acceleration_ = 0.0;
      RCLCPP_WARN(
        this->get_logger(), "[Robot%d] Odometry dt too small: %.9f",
        robot_index_, dt);
    } else {
      this->acceleration_ = (this->speed_ - this->old_speed_) / dt;
    }
  } else {
    this->acceleration_ = 0;
  }
  this->yaw_rate_ = msg->twist.twist.angular.z;

  if (std::abs(this->speed_) <
    0.01)    // Use threshold instead of exact comparison
  {
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
    "[Robot%d] Odom update | Speed: %.2f m/s | Accel: %.2f m/s² | "
    "Yaw rate: %.2f rad/s",
    robot_index_, this->speed_, this->acceleration_,
    this->yaw_rate_);
}

void CamCommunication::pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_DEBUG(
    this->get_logger(), "[Robot%d] Received pose update",
    robot_index_);
  this->longitude_ = msg->pose.position.x;
  this->latitude_ = msg->pose.position.y;
  this->altitude_ = msg->pose.position.z;

  tf2::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w);
  quat.normalize();
  tf2::Matrix3x3 matrix(quat);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  matrix.getRPY(roll, pitch, yaw);

  this->heading_ = yaw;
  RCLCPP_DEBUG(
    this->get_logger(),
    "[Robot%d] Pose update | Pos: (%.2f, %.2f, %.2f) | Heading: %.2f°",
    robot_index_, this->longitude_, this->latitude_, this->altitude_,
    this->heading_ * 180.0 / M_PI);
}
