#include "auna_omnet/omnet_transmitter.hpp"

#include <cmath>       // For std::isnan
#include <functional>  // For std::hash
#include <stdexcept>   // For std::invalid_argument, std::out_of_range
#include <string>      // For std::string, std::stoul

// Create subscribers to get robot data and publisher to send to Artery and OMNeT++
OmnetTransmitter::OmnetTransmitter(const std::string & robot_name) : Node("omnet_transmitter_node")
{
  pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "global_pose", 2,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_callback(msg); });
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

  timer = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { cam_callback(); });  // Set a default timer period
  publisher = this->create_publisher<etsi_its_cam_msgs::msg::CAM>("cam_out", 2);

  this->robot_name_ = robot_name;
}

// Send CAM data to Artery and OMNeT++ with most recently received data
void OmnetTransmitter::cam_callback()
{
  etsi_its_cam_msgs::msg::CAM msg;

  // ITS PDU Header
  msg.header.protocol_version = 2;
  msg.header.message_id = etsi_its_cam_msgs::msg::ItsPduHeader::MESSAGE_ID_CAM;

  uint32_t station_id_val = 0;  // Default
  if (!robot_name_.empty()) {
    try {
      std::size_t last_char_pos = robot_name_.find_last_not_of("0123456789");
      if (last_char_pos != std::string::npos && last_char_pos < robot_name_.length() - 1) {
        station_id_val = static_cast<uint32_t>(std::stoul(robot_name_.substr(last_char_pos + 1)));
      } else if (robot_name_.find_first_not_of("0123456789") == std::string::npos) {  // If
                                                                                      // robot_name_
                                                                                      // is purely
                                                                                      // numeric
        station_id_val = static_cast<uint32_t>(std::stoul(robot_name_));
      } else {  // Fallback to hash if not ending in a number or not purely numeric
        std::hash<std::string> hasher;
        station_id_val = static_cast<uint32_t>(hasher(robot_name_));
        RCLCPP_DEBUG(
          this->get_logger(),
          "Robot name '%s' not purely numeric or not ending in number, using hash %u for station "
          "ID.",
          robot_name_.c_str(), station_id_val);
      }
    } catch (const std::invalid_argument & ia) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid argument for station ID from robot_name: '%s'. Using default 0. Error: %s",
        robot_name_.c_str(), ia.what());
      station_id_val = 0;
    } catch (const std::out_of_range & oor) {
      RCLCPP_WARN(
        this->get_logger(),
        "Out of range for station ID from robot_name: '%s'. Using default 0. Error: %s",
        robot_name_.c_str(), oor.what());
      station_id_val = 0;
    }
  }
  msg.header.station_id.value = station_id_val;

  // CAM Payload
  uint64_t now_ns = this->get_clock()->now().nanoseconds();
  msg.cam.generation_delta_time.value = static_cast<uint16_t>((now_ns / 1000000) % 65536);

  // Basic Container
  auto & basic_container = msg.cam.cam_parameters.basic_container;
  basic_container.station_type.value = etsi_its_cam_msgs::msg::StationType::PASSENGER_CAR;

  // Original scaling: longitude_ * scale_factor_ * 10 for a field unit of 0.1m
  // New field unit for longitude is 1/10 microdegree. Assuming original values were Cartesian
  // meters. This mapping is likely incorrect for standard ETSI CAM if longitude_ was meters.
  // Preserving original numeric transformation for now.
  basic_container.reference_position.longitude.value =
    static_cast<int32_t>(this->longitude_ * scale_factor_ * 10.0f);
  basic_container.reference_position.latitude.value =
    static_cast<int32_t>(this->latitude_ * scale_factor_ * 10.0f);
  basic_container.reference_position.altitude.altitude_value.value =
    static_cast<int32_t>(this->altitude_ * scale_factor_ * 100.0f);  // Original for 0.01m unit
  basic_container.reference_position.altitude.altitude_confidence.value =
    etsi_its_cam_msgs::msg::AltitudeConfidence::UNAVAILABLE;
  basic_container.reference_position.position_confidence_ellipse.semi_major_confidence.value =
    etsi_its_cam_msgs::msg::SemiAxisLength::UNAVAILABLE;
  basic_container.reference_position.position_confidence_ellipse.semi_minor_confidence.value =
    etsi_its_cam_msgs::msg::SemiAxisLength::UNAVAILABLE;
  basic_container.reference_position.position_confidence_ellipse.semi_major_orientation.value =
    etsi_its_cam_msgs::msg::HeadingValue::UNAVAILABLE;

  // High Frequency Container
  auto & hf_container = msg.cam.cam_parameters.high_frequency_container;
  hf_container.choice =
    etsi_its_cam_msgs::msg::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
  auto & vhf = hf_container.basic_vehicle_container_high_frequency;

  // Original: heading_ * 10 for a field unit of 0.1 degree. this->heading_ is [0,360] degrees.
  vhf.heading.heading_value.value = static_cast<uint16_t>(this->heading_ * 10.0f);
  vhf.heading.heading_confidence.value = etsi_its_cam_msgs::msg::HeadingConfidence::UNAVAILABLE;

  // Original: speed_ * scale_factor_ * 100 for a field unit of 0.01 m/s. this->speed_ is m/s.
  vhf.speed.speed_value.value = static_cast<uint16_t>(this->speed_ * scale_factor_ * 100.0f);
  vhf.speed.speed_confidence.value = etsi_its_cam_msgs::msg::SpeedConfidence::UNAVAILABLE;

  // Original: speed_ < 0. this->speed_ is always non-negative from odom_callback.
  if (this->speed_ > 0.01f) {
    vhf.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::FORWARD;
  } else {
    vhf.drive_direction.value = etsi_its_cam_msgs::msg::DriveDirection::UNAVAILABLE;
  }

  // Original: 0.49 * scale_factor_ for a field unit of m. New unit is 0.1m.
  vhf.vehicle_length.vehicle_length_value.value =
    static_cast<uint16_t>((0.49f * scale_factor_) * 10.0f);
  vhf.vehicle_length.vehicle_length_confidence_indication.value =
    etsi_its_cam_msgs::msg::VehicleLengthConfidenceIndication::UNAVAILABLE;

  // Original: 0.18 * scale_factor_ for a field unit of m. New unit is 0.1m.
  vhf.vehicle_width.value = static_cast<uint16_t>((0.18f * scale_factor_) * 10.0f);

  // Original: acceleration_ * scale_factor_ * 10 for a field unit of 0.1m/s^2. this->acceleration_
  // is m/s^2.
  vhf.longitudinal_acceleration.longitudinal_acceleration_value.value =
    static_cast<int16_t>(this->acceleration_ * scale_factor_ * 10.0f);
  vhf.longitudinal_acceleration.longitudinal_acceleration_confidence.value =
    etsi_its_cam_msgs::msg::AccelerationConfidence::UNAVAILABLE;

  // Original: (curvature_ / scale_factor_) for a field unit of 1/m. New unit is 1/10000 per meter.
  if (std::isnan(this->curvature_)) {
    vhf.curvature.curvature_value.value = etsi_its_cam_msgs::msg::CurvatureValue::UNAVAILABLE;
  } else {
    vhf.curvature.curvature_value.value =
      static_cast<int16_t>((this->curvature_ / scale_factor_) * 10000.0f);
  }
  vhf.curvature.curvature_confidence.value =
    etsi_its_cam_msgs::msg::CurvatureConfidence::UNAVAILABLE;
  vhf.curvature_calculation_mode.value =
    etsi_its_cam_msgs::msg::CurvatureCalculationMode::YAW_RATE_USED;

  // Original: yaw_rate_ * 100 for a field unit of 0.01degree/s. this->yaw_rate_ is degree/s.
  if (std::isnan(this->yaw_rate_)) {
    vhf.yaw_rate.yaw_rate_value.value = etsi_its_cam_msgs::msg::YawRateValue::UNAVAILABLE;
  } else {
    vhf.yaw_rate.yaw_rate_value.value = static_cast<int16_t>(this->yaw_rate_ * 100.0f);
  }
  vhf.yaw_rate.yaw_rate_confidence.value = etsi_its_cam_msgs::msg::YawRateConfidence::UNAVAILABLE;

  publisher->publish(msg);
}

// Receive and save the odometry data for speed, acceleration, yaw rate and curvature
void OmnetTransmitter::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  float old_speed = this->speed_;
  this->speed_ = sqrt(
    pow(msg->twist.twist.linear.x, 2) +
    pow(msg->twist.twist.linear.y, 2));  // absolute speed without direction
  this->acceleration_ = (this->speed_ - old_speed) / (1000 / publish_period_);
  this->yaw_rate_ = msg->twist.twist.angular.z;    // yaw rate in radians/s
  this->yaw_rate_ = this->yaw_rate_ * 180 / M_PI;  // yaw rate in degree/s
  this->curvature_ = this->yaw_rate_ / std::max(0.01f, this->speed_);
}

// Receive and save the robot pose data
void OmnetTransmitter::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  this->longitude_ = msg->pose.position.x;
  this->latitude_ = msg->pose.position.y;
  this->altitude_ = msg->pose.position.z;

  // Determine yaw from orientation quaternion
  tf2::Quaternion quat(
    msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
    msg->pose.orientation.w);
  quat.normalize();
  tf2::Matrix3x3 matrix(quat);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  matrix.getRPY(roll, pitch, yaw);

  this->heading_ = yaw;                          // get heading in radians
  this->heading_ = this->heading_ * 180 / M_PI;  // get heading in degree
  this->heading_ =
    this->heading_ - 360 * floor(this->heading_ / 360);  // get heading in interval [0,360]
}