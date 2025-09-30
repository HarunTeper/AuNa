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


#include "auna_cacc/cacc_controller.hpp"

#include <rclcpp/logging.hpp>

#include <etsi_its_msgs_utils/impl/cam/cam_getters_common.h>

#include <cmath>
#include <fstream>  // Added for file logging

CaccController::CaccController()
: Node("cacc_controller")
{
  // Add startup logging
  RCLCPP_INFO(this->get_logger(), "Starting CACC controller, waiting for required messages");

  // Track if first messages have been received
  first_cam_received_ = false;
  first_odom_received_ = false;
  first_pose_received_ = false;

  // Initialize data logging
  this->declare_parameter("enable_data_logging", true);
  this->declare_parameter("log_file_path", "/home/vscode/workspace/cacc_log.csv");

  enable_data_logging_ = false;
  log_file_path_ = this->get_parameter("log_file_path").as_string();

  if (enable_data_logging_) {
    RCLCPP_INFO(this->get_logger(), "Data logging enabled. Writing to: %s", log_file_path_.c_str());
    log_file_.open(log_file_path_, std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
      // Write CSV header (including new debug columns)
      log_file_
        << "timestamp,leader_x,leader_y,leader_velocity,leader_acceleration,leader_curvature,"
        << "follower_x,follower_y,follower_velocity,follower_acceleration,"
        << "desired_distance,actual_distance,distance_error,"
        << "z1,z2,z3,z4,"
        << "alpha,s,"  // Debug values start here
        << "invGam1,invGam2,invGam3,invGam4,"
        << "inP1_pos_err,inP1_vel_err,inP1_geom_vel,inP1_yaw_rate,"
        << "inP2_pos_err,inP2_vel_err,inP2_geom_vel,inP2_yaw_rate,"
        << "commanded_accel,commanded_velocity,commanded_angular" << std::endl;
      log_file_.flush();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_file_path_.c_str());
      enable_data_logging_ = false;
    }
  }

  // Add topic name debug info
  RCLCPP_INFO(this->get_logger(), "Subscribing to topics: cam_filtered, odom, global_pose");

  sub_cam_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
    "cam_filtered", 2, [this](const etsi_its_cam_msgs::msg::CAM::SharedPtr msg) {
      if (!first_cam_received_) {
        RCLCPP_INFO(this->get_logger(), "Received first CAM message");
        first_cam_received_ = true;
      }
      this->cam_callback(msg);
    });
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered_ekf", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if (!first_odom_received_) {
        RCLCPP_INFO(this->get_logger(), "Received first odom message");
        first_odom_received_ = true;
      }
      this->odom_callback(msg);
    });
  sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "global_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      if (!first_pose_received_) {
        RCLCPP_INFO(this->get_logger(), "Received first global_pose message");
        first_pose_received_ = true;
      }
      this->pose_callback(msg);
    });

  pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel/cacc", 1);
  RCLCPP_INFO(
    this->get_logger(), "Created cmd_vel publisher on topic: %s", pub_cmd_vel->get_topic_name());
  pub_x_lookahead_point_ = this->create_publisher<std_msgs::msg::Float64>("cacc/lookahead/x", 1);
  pub_y_lookahead_point_ = this->create_publisher<std_msgs::msg::Float64>("cacc/lookahead/y", 1);
  pub_waypoints_pose_array_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("cacc/waypoints", 1);
  pub_closest_pose_waypoint_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("cacc/closest_pose_waypoint", 1);
  pub_closest_cam_waypoint_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("cacc/closest_cam_waypoint", 1);
  pub_target_waypoint_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("cacc/target_waypoint", 1);
  pub_cacc_pose_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("cacc/lookahead_pose", 1);

  client_set_standstill_distance_ = this->create_service<auna_msgs::srv::SetFloat64>(
    "cacc/set_standstill_distance",
    [this](
      const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
      std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response) {
      this->set_standstill_distance(request, response);
    });
  client_set_time_gap_ = this->create_service<auna_msgs::srv::SetFloat64>(
    "cacc/set_time_gap", [this](
      const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
      std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response) {
      this->set_time_gap(request, response);
    });
  client_set_cacc_enable_ = this->create_service<auna_msgs::srv::SetBool>(
    "/cacc/set_cacc_enable", [this](
      const std::shared_ptr<auna_msgs::srv::SetBool::Request> request,
      std::shared_ptr<auna_msgs::srv::SetBool::Response> response) {
      this->set_cacc_enable(request, response);
    });
  client_set_target_velocity_ = this->create_service<auna_msgs::srv::SetFloat64>(
    "/cacc/set_target_velocity",
    [this](
      const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
      std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response) {
      this->set_target_velocity(request, response);
    });
  client_set_extra_distance_ = this->create_service<auna_msgs::srv::SetFloat64>(
    "/cacc/set_extra_distance",
    [this](
      const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
      std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response) {
      this->set_extra_distance(request, response);
    });

  this->declare_parameter("standstill_distance", 1.25);
  this->declare_parameter("time_gap", 0.25);
  this->declare_parameter("kp", 1.0);
  this->declare_parameter("kd", 1.0);
  this->declare_parameter("max_velocity", 1.0);
  this->declare_parameter("frequency", 50);
  this->declare_parameter("use_waypoints", false);
  this->declare_parameter("waypoint_file", "/home/$USER/waypoints.txt");
  this->declare_parameter("target_velocity", 1.0);
  this->declare_parameter("curvature_lookahead", 1.0);
  this->declare_parameter("extra_distance", 0.0);

  // use params_
  params_.standstill_distance = this->get_parameter("standstill_distance").as_double();
  params_.time_gap = this->get_parameter("time_gap").as_double();
  params_.kp = this->get_parameter("kp").as_double();
  params_.max_velocity = this->get_parameter("max_velocity").as_double();
  params_.frequency = this->get_parameter("frequency").as_int();
  params_.use_waypoints = this->get_parameter("use_waypoints").as_bool();
  params_.waypoint_file = this->get_parameter("waypoint_file").as_string();
  params_.target_velocity = this->get_parameter("target_velocity").as_double();
  params_.curvature_lookahead = this->get_parameter("curvature_lookahead").as_double();
  params_.extra_distance = this->get_parameter("extra_distance").as_double();

  dt_ = 1.0 / params_.frequency;
  auto_mode_ = false;
  auto_mode_ready_ = false;
  cacc_ready_ = false;

  client_set_auto_mode_ = this->create_service<auna_msgs::srv::SetBool>(
    "cacc/set_auto_mode", [this](
      const std::shared_ptr<auna_msgs::srv::SetBool::Request> request,
      std::shared_ptr<auna_msgs::srv::SetBool::Response> response) {
      this->set_auto_mode(request, response);
    });

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / params_.frequency), [this]() {this->timer_callback();});
  setup_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), [this]() {this->setup_timer_callback();});
  timer_->cancel();

  if (params_.use_waypoints) {
    read_waypoints_from_csv();
  }

  dyn_params_handler_ = this->add_on_set_parameters_callback(
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
      return this->dynamicParametersCallback(parameters);
    });

  // Add cleanup for file logging
  rclcpp::on_shutdown(
    [this]() {
      if (enable_data_logging_ && log_file_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Closing log file");
        log_file_.close();
      }
    });
}

void CaccController::read_waypoints_from_csv()
{
  std::string file_path = this->get_parameter("waypoint_file").as_string();

  std::ifstream file(file_path, std::ifstream::in);
  std::string line;
  if (file.is_open()) {
    waypoints_x_.clear();  // Clear the existing waypoints_x_ vector
    waypoints_y_.clear();  // Clear the existing waypoints_y_ vector

    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string x, y;
      std::getline(iss, x, ',');
      std::getline(iss, y, ',');
      try {
        double x_value = std::stod(x);
        double y_value = std::stod(y);

        waypoints_x_.push_back(x_value);
        waypoints_y_.push_back(y_value);
      } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(
          this->get_logger(), "Invalid waypoint format in CSV file: %s", file_path.c_str());
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", file_path.c_str());
    return;
  }
  file.close();

  // Calculate yaw for each waypoint
  waypoints_yaw_.clear();  // Clear the existing waypoints_yaw_ vector

  size_t num_waypoints = waypoints_x_.size();
  for (size_t i = 0; i < num_waypoints; ++i) {
    size_t prev_index = (i == 0) ? (num_waypoints - 1) : (i - 1);
    size_t next_index = (i + 1) % num_waypoints;

    double next_x = waypoints_x_[next_index];
    double next_y = waypoints_y_[next_index];

    double prev_x = waypoints_x_[prev_index];
    double prev_y = waypoints_y_[prev_index];

    double yaw = std::atan2(next_y - prev_y, next_x - prev_x);
    waypoints_yaw_.push_back(yaw);
  }

  // Create and publish a pose array of all waypoints
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = this->get_clock()->now();
  pose_array.header.frame_id = "map";  // Use appropriate frame_id

  pose_array.poses.resize(num_waypoints);

  for (size_t i = 0; i < num_waypoints; ++i) {
    // Set position
    pose_array.poses[i].position.x = waypoints_x_[i];
    pose_array.poses[i].position.y = waypoints_y_[i];
    pose_array.poses[i].position.z = 0.0;

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, waypoints_yaw_[i]);

    // Set orientation
    pose_array.poses[i].orientation.x = q.x();
    pose_array.poses[i].orientation.y = q.y();
    pose_array.poses[i].orientation.z = q.z();
    pose_array.poses[i].orientation.w = q.w();
  }

  // Publish the pose array
  pub_waypoints_pose_array_->publish(pose_array);
  RCLCPP_INFO(this->get_logger(), "Published pose array with %zu waypoints", num_waypoints);
}

void CaccController::setup_timer_callback()
{
  // if all last messages exist, start timer
  if (last_cam_msg_ != nullptr && last_odom_msg_ != nullptr && last_pose_msg_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "CACC controller ready");
    setup_timer_->cancel();
    cacc_ready_ = true;
  }
  if (last_odom_msg_ != nullptr && last_pose_msg_ != nullptr) {
    if (!auto_mode_ready_) {
      RCLCPP_INFO(this->get_logger(), "Auto mode ready");
      auto_mode_ready_ = true;
    }
  }
  // Add debug log to show we're still waiting
  RCLCPP_DEBUG(
    this->get_logger(), "Waiting for messages - CAM: %s, Odom: %s, Pose: %s",
    (last_cam_msg_ != nullptr ? "received" : "waiting"),
    (last_odom_msg_ != nullptr ? "received" : "waiting"),
    (last_pose_msg_ != nullptr ? "received" : "waiting"));
}

void CaccController::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
{
  RCLCPP_DEBUG(
    this->get_logger(), "cam_callback - Generation Delta Time: %u",
    msg->cam.generation_delta_time.value);
  cam_x_ = msg->cam.cam_parameters.basic_container.reference_position.longitude.value / 10000000.0;
  cam_y_ = msg->cam.cam_parameters.basic_container.reference_position.latitude.value / 10000000.0;
  cam_velocity_ = msg->cam.cam_parameters.high_frequency_container
    .basic_vehicle_container_high_frequency.speed.speed_value.value /
    100.0;                // Convert from 0.01 m/s to m/s

  // Calculate acceleration
  if (last_cam_msg_ == nullptr) {
    cam_acceleration_ = 0.0;
  } else {
    // Use generationDeltaTime for time difference with proper wraparound handling
    uint16_t current_time = msg->cam.generation_delta_time.value;
    uint16_t previous_time = last_cam_msg_->cam.generation_delta_time.value;

    double dt;
    if (current_time < previous_time) {
      dt = (65536 + current_time - previous_time) / 1000.0;
      RCLCPP_DEBUG(
        this->get_logger(), "Generation Delta Time wraparound detected: %u → %u", previous_time,
        current_time);
    } else {
      dt = (current_time - previous_time) / 1000.0;
    }

    RCLCPP_DEBUG(this->get_logger(), "Generation Delta Time difference: %.4f", dt);

    dt = std::min(dt, 0.1);

    if (dt < 0.001) {
      cam_acceleration_ = 0.0;
    } else {
      cam_acceleration_ = (cam_velocity_ - last_cam_velocity_) / dt;
    }
  }
  last_cam_msg_ = msg;

  // Properly handle the heading value with bounds checking
  uint16_t heading_value = msg->cam.cam_parameters.high_frequency_container
    .basic_vehicle_container_high_frequency.heading.heading_value.value;

  if (heading_value == etsi_its_cam_msgs::msg::HeadingValue::UNAVAILABLE) {
    RCLCPP_WARN(this->get_logger(), "Heading value is UNAVAILABLE");
    cam_yaw_ = 0.0;  // Default to 0 when unavailable
  } else {
    // Make sure heading is within 0-3600 range (0-360 degrees)
    heading_value = heading_value % 3600;
    double heading_degrees = heading_value / 10.0;  // Convert from 0.1 degrees to degrees
    cam_yaw_ = heading_degrees * (M_PI / 180.0);    // Convert from degrees to radians
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "Received CAM - Speed: %.2f m/s, Accel: %.2f m/s², Yaw Rate: %.2f deg/s, Curvature: %.4f, "
    "Station ID: %d",
    cam_velocity_, cam_acceleration_, cam_yaw_rate_ * 180 / M_PI, cam_curvature_,
    msg->header.station_id.value);

  RCLCPP_DEBUG(
    this->get_logger(), "Heading: %.2f degrees (%.2f radians)", heading_value / 10.0, cam_yaw_);

  last_cam_velocity_ = cam_velocity_;

  if (
    msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.yaw_rate
    .yaw_rate_value.value != etsi_its_cam_msgs::msg::YawRateValue::UNAVAILABLE &&
    cam_velocity_ > 0.1)
  {
    double raw_yaw = msg->cam.cam_parameters.high_frequency_container
      .basic_vehicle_container_high_frequency.yaw_rate.yaw_rate_value.value;
    cam_yaw_rate_ = (raw_yaw / 100.0) * (M_PI / 180.0);  // Convert from 0.01 degrees/s to radians/s
    RCLCPP_DEBUG(
      this->get_logger(), "Yaw rate conversion: raw=%.2f → %.4f rad/s", raw_yaw, cam_yaw_rate_);
    cam_curvature_ = cam_yaw_rate_ / cam_velocity_;
  } else {
    cam_yaw_rate_ = 0.0;
    cam_curvature_ = 0.0;
  }
}

void CaccController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_velocity_ = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2)) *
    ((msg->twist.twist.linear.x < 0) ? -1 : 1);
  if (last_odom_msg_ == nullptr) {
    odom_acceleration_ = 0.0;
  } else {
    double dt = msg->header.stamp.sec - last_odom_msg_->header.stamp.sec +
      (msg->header.stamp.nanosec - last_odom_msg_->header.stamp.nanosec) / 1e9;

    dt = std::min(dt, 0.01);
    odom_acceleration_ = (odom_velocity_ - last_odom_velocity_) / dt;
  }
  last_odom_msg_ = msg;
  last_odom_velocity_ = odom_velocity_;
  odom_yaw_rate_ = msg->twist.twist.angular.z;

  if (odom_velocity_ == 0) {
    odom_curvature_ = 0;
  } else {
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

void CaccController::update_waypoint_following()
{
  int closest_pose_waypoint_index;
  {
    double closest_distance_squared = std::numeric_limits<double>::max();
    int num_waypoints = waypoints_x_.size();

    for (int i = 0; i < num_waypoints; ++i) {
      double dx = waypoints_x_[i] - pose_x_;
      double dy = waypoints_y_[i] - pose_y_;
      double distance_squared = dx * dx + dy * dy;

      if (distance_squared < closest_distance_squared) {
        closest_distance_squared = distance_squared;
        closest_pose_waypoint_index = i;
      }
    }
  }
  // Publish the closest waypoint
  publish_waypoint_pose(pub_closest_pose_waypoint_, closest_pose_waypoint_index);

  // Check if we have received any CAM messages
  int closest_cam_waypoint_index = -1;  // Default invalid index
  if (last_cam_msg_ != nullptr) {
    // Find the closest waypoint to the leader (CAM) position
    double closest_distance_squared = std::numeric_limits<double>::max();
    int num_waypoints = waypoints_x_.size();

    for (int i = 0; i < num_waypoints; ++i) {
      double dx = waypoints_x_[i] - cam_x_;
      double dy = waypoints_y_[i] - cam_y_;
      double distance_squared = dx * dx + dy * dy;

      if (distance_squared < closest_distance_squared) {
        closest_distance_squared = distance_squared;
        closest_cam_waypoint_index = i;
      }
    }

    // Publish the closest CAM waypoint (only if we found a valid one)
    if (closest_cam_waypoint_index >= 0) {
      publish_waypoint_pose(pub_closest_cam_waypoint_, closest_cam_waypoint_index);
    }
  }

  // This limits acceleration by clamping the velocity difference and uses the adjusted velocity for
  // a smoother time-gap distance calculation
  double velocity_diff;
  if (last_cam_msg_ == nullptr) {
    // No leader vehicle data available, use target velocity parameter
    velocity_diff = params_.target_velocity - odom_velocity_;
    RCLCPP_DEBUG(
      this->get_logger(), "No CAM data: Using target velocity parameter (%.2f m/s)",
      params_.target_velocity);
  } else {
    // Use leader vehicle's velocity from CAM message
    velocity_diff = cam_velocity_ - odom_velocity_;
    RCLCPP_DEBUG(
      this->get_logger(), "Using CAM velocity: Leader=%.2f m/s, Follower=%.2f m/s, Diff=%.2f m/s",
      cam_velocity_, odom_velocity_, velocity_diff);
  }

  // Clamp velocity difference to limit acceleration/deceleration rate
  const double MAX_ACCEL_RATE = 0.1;  // m/s per control cycle
  double clamped_diff = std::max(std::min(velocity_diff, MAX_ACCEL_RATE), -MAX_ACCEL_RATE);
  double adjusted_velocity = odom_velocity_ + clamped_diff;

  // Ensure adjusted velocity is never negative
  adjusted_velocity = std::max(0.0, adjusted_velocity);

  // Calculate target distance using adjusted velocity
  double target_distance = params_.standstill_distance + params_.time_gap * adjusted_velocity;

  // Initialize target waypoint calculation
  int target_waypoint_index = closest_pose_waypoint_index;
  int num_waypoints = waypoints_x_.size();
  double accumulated_distance = 0.0;
  bool target_reached = false;

  // If we have CAM data and a valid CAM waypoint, consider it as a maximum bound
  bool has_cam_limit = (last_cam_msg_ != nullptr && closest_cam_waypoint_index >= 0);

  // Find target waypoint by accumulating distance along path
  for (int i = 0; i < num_waypoints && !target_reached; i++) {
    int current_index = (closest_pose_waypoint_index + i) % num_waypoints;
    int next_index = (closest_pose_waypoint_index + i + 1) % num_waypoints;

    // Calculate distance between consecutive waypoints
    double segment_dx = waypoints_x_[next_index] - waypoints_x_[current_index];
    double segment_dy = waypoints_y_[next_index] - waypoints_y_[current_index];
    double segment_distance = std::hypot(segment_dx, segment_dy);

    // Check if adding this segment exceeds our target distance
    if (accumulated_distance + segment_distance >= target_distance) {
      target_waypoint_index = next_index;
      target_reached = true;
      RCLCPP_DEBUG(
        this->get_logger(), "Target waypoint found at distance %.2f m (target was %.2f m)",
        accumulated_distance + segment_distance, target_distance);
    }
    // If we have CAM data, check if we've reached the leader's position
    else if (has_cam_limit) {
      if (next_index == closest_cam_waypoint_index) {
        target_waypoint_index = next_index;
        target_reached = true;
        RCLCPP_DEBUG(this->get_logger(), "Reached leader's position at waypoint %d", next_index);
      }
    }

    // Add this segment to our accumulated distance

    accumulated_distance += segment_distance;

    // If we haven't reached the target yet, update the index to the next waypoint
    if (!target_reached) {
      target_waypoint_index = next_index;
    }
  }
  // Publish the target waypoint
  publish_waypoint_pose(pub_target_waypoint_, target_waypoint_index);

  control_x_ = waypoints_x_[target_waypoint_index];
  control_y_ = waypoints_y_[target_waypoint_index];
  control_yaw_ = waypoints_yaw_[target_waypoint_index];

  // Calculate cam_yaw_rate_ and cam_curvature_
  int curr_index = target_waypoint_index;
  double current_yaw = waypoints_yaw_[curr_index];

  // Calculate next index based on lookahead distance in meters
  int lookahead_index = target_waypoint_index;
  double accumulated_lookahead_distance = 0.0;
  {
    int num_waypoints = waypoints_x_.size();

    // Accumulate distance until we reach the specified lookahead distance
    for (int i = 0; i < num_waypoints; i++) {
      int current_index = (target_waypoint_index + i) % num_waypoints;
      int next_waypoint_index = (target_waypoint_index + i + 1) % num_waypoints;

      // Calculate distance between consecutive waypoints
      double segment_dx = waypoints_x_[next_waypoint_index] - waypoints_x_[current_index];
      double segment_dy = waypoints_y_[next_waypoint_index] - waypoints_y_[current_index];
      double segment_distance = std::hypot(segment_dx, segment_dy);

      // Add this segment to our accumulated distance
      accumulated_lookahead_distance += segment_distance;
      lookahead_index = next_waypoint_index;

      // If we've accumulated enough distance, we're done
      if (accumulated_lookahead_distance >= params_.curvature_lookahead) {
        break;
      }
    }
  }
  double next_yaw = waypoints_yaw_[lookahead_index];

  // yaw difference in the range of -pi to pi
  double yaw_difference;
  if (next_yaw - current_yaw > M_PI) {
    yaw_difference = next_yaw - current_yaw - 2 * M_PI;
  } else if (next_yaw - current_yaw < -M_PI) {
    yaw_difference = next_yaw - current_yaw + 2 * M_PI;
  } else {
    yaw_difference = next_yaw - current_yaw;
  }

  // Calculate the required time to reach the n-th next waypoint
  // Select appropriate target velocity based on CAM message availability
  double target_vel_for_lookahead;
  if (last_cam_msg_ != nullptr) {
    // Use leader's velocity with a small safety margin
    target_vel_for_lookahead = cam_velocity_;
    RCLCPP_DEBUG(
      this->get_logger(), "Using leader velocity for lookahead calculations: %.2f m/s",
      target_vel_for_lookahead);
  } else {
    // No leader data, use configured target velocity
    target_vel_for_lookahead = params_.target_velocity;
    RCLCPP_DEBUG(
      this->get_logger(), "Using configured target velocity for lookahead: %.2f m/s",
      target_vel_for_lookahead);
  }

  if (target_vel_for_lookahead < 0.1) {
    control_yaw_rate_ = 0.0;
  } else {
    double required_time = accumulated_lookahead_distance / target_vel_for_lookahead;
    control_yaw_rate_ = yaw_difference / required_time;
  }

  // Calculate control velocity considering leader vehicle when available
  double base_velocity;
  if (last_cam_msg_ != nullptr) {
    // Use leader's velocity as the base reference
    base_velocity = cam_velocity_;
    RCLCPP_DEBUG(this->get_logger(), "Using leader velocity as base: %.2f m/s", base_velocity);
  } else {
    // No leader data, use configured target velocity
    base_velocity = params_.target_velocity;
    RCLCPP_DEBUG(
      this->get_logger(), "Using configured target velocity as base: %.2f m/s", base_velocity);
  }

  // Apply cornering adjustment based on yaw rate
  // Reduce velocity in curves proportionally to the yaw rate
  double cornering_factor = std::max(0.0, 1.0 - std::abs(control_yaw_rate_) / 1.25);

  // Apply cornering slowdown only to a portion of the velocity to prevent excessive slowdowns
  double min_cornering_velocity =
    base_velocity * 0.7;  // Don't go below 70% of target speed in curves
  control_velocity_ = std::max(min_cornering_velocity, base_velocity * cornering_factor);

  // Respect the maximum velocity limit
  control_velocity_ = std::min(control_velocity_, params_.max_velocity);

  // Calculate acceleration for logging
  control_acceleration_ = (control_velocity_ - control_last_velocity_) / dt_;
  control_last_velocity_ = control_velocity_;

  // TODO check if target_velocity or control velocity should be used
  // cam curvature depending on auto_mode
  // Prevent division by zero when calculating control curvature
  if (std::abs(control_velocity_) < 0.01) {
    control_curvature_ = 0.0;
  } else {
    control_curvature_ = control_yaw_rate_ / params_.target_velocity;
  }
}

void CaccController::timer_callback()
{
  if (params_.use_waypoints) {
    update_waypoint_following();
  }

  RCLCPP_DEBUG(this->get_logger(), "=== CACC Following Status ===");

  // Log vehicle states - key information about both vehicles
  RCLCPP_DEBUG(this->get_logger(), "Vehicle States:");
  RCLCPP_DEBUG(
    this->get_logger(), "  Leader: velocity=%.2f m/s, acceleration=%.2f m/s², curvature=%.4f 1/m",
    cam_velocity_, cam_acceleration_, cam_curvature_);
  RCLCPP_DEBUG(
    this->get_logger(), "  Follower: velocity=%.2f m/s, acceleration=%.2f m/s²", odom_velocity_,
    odom_acceleration_);

  // Log distance parameters
  double distance_term = params_.standstill_distance + params_.time_gap * odom_velocity_;
  RCLCPP_DEBUG(this->get_logger(), "Distance Parameters:");
  RCLCPP_DEBUG(
    this->get_logger(),
    "  Desired spacing: %.2f m (standstill=%.2f + time_gap=%.2f * velocity=%.2f)", distance_term,
    params_.standstill_distance, params_.time_gap, odom_velocity_);

  // Calculate actual distance between vehicles
  double dx = cam_x_ - pose_x_;
  double dy = cam_y_ - pose_y_;
  double actual_distance = std::hypot(dx, dy);
  RCLCPP_DEBUG(
    this->get_logger(), "  Actual distance: %.2f m (error: %.2f m)", actual_distance,
    actual_distance - distance_term);

  // Calculate s_ (distance along curved path)
  if (control_curvature_ < 0.01 && control_curvature_ > -0.01) {
    double term1_s = 0.5 * pow(distance_term, 2) * control_curvature_;
    double term2_s = 0.125 * pow(distance_term, 4) * pow(control_curvature_, 3);
    s_ = term1_s - term2_s;
  } else {
    double inner_term = 1 + pow(control_curvature_, 2) * pow(distance_term, 2);
    s_ = (-1 + sqrt(inner_term)) / control_curvature_;
  }

  // Calculate alpha_ (angle adjustment for curved path)
  alpha_ = atan(control_curvature_ * distance_term);

  // Store debug values for logging
  dbg_alpha_ = alpha_;  // Store for logging
  dbg_s_ = s_;          // Store for logging

  RCLCPP_DEBUG(this->get_logger(), "Curved Path Adjustment:");
  RCLCPP_DEBUG(
    this->get_logger(), "  Path curvature: %.4f 1/m, s: %.2f m, alpha: %.2f rad",
    control_curvature_, dbg_s_, dbg_alpha_);

  x_lookahead_point_ = control_x_ + s_ * sin(control_yaw_);
  y_lookahead_point_ = control_y_ - s_ * cos(control_yaw_);

  // Publish lookahead using publish_waypoint_pose
  geometry_msgs::msg::PoseStamped lookahead_point;
  lookahead_point.header.stamp = this->get_clock()->now();
  lookahead_point.header.frame_id = "map";  // Use appropriate frame_id
  lookahead_point.pose.position.x = x_lookahead_point_;
  lookahead_point.pose.position.y = y_lookahead_point_;
  lookahead_point.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, control_yaw_);
  lookahead_point.pose.orientation.x = q.x();
  lookahead_point.pose.orientation.y = q.y();
  lookahead_point.pose.orientation.z = q.z();
  lookahead_point.pose.orientation.w = q.w();
  pub_cacc_pose_->publish(lookahead_point);

  // Calculate error states
  z_1_ = control_x_ - pose_x_ + s_ * sin(control_yaw_) - distance_term * cos(pose_yaw_);
  z_2_ = control_y_ - pose_y_ - s_ * cos(control_yaw_) - distance_term * sin(pose_yaw_);
  z_3_ = control_velocity_ * cos(control_yaw_) - odom_velocity_ * cos(pose_yaw_ + alpha_);
  z_4_ = control_velocity_ * sin(control_yaw_) - odom_velocity_ * sin(pose_yaw_ + alpha_);

  RCLCPP_INFO(this->get_logger(), "Error States:");
  RCLCPP_INFO(
    this->get_logger(), "  Position errors - longitudinal: %.2f m, lateral: %.2f m", z_1_, z_2_);
  RCLCPP_INFO(
    this->get_logger(), "  Velocity errors - longitudinal: %.2f m/s, lateral: %.2f m/s", z_3_,
    z_4_);

  // Calculate control matrix inverse (invGam calculations)
  double term1 = params_.standstill_distance + params_.time_gap * odom_velocity_;
  double term2 = params_.time_gap - params_.time_gap * sin(alpha_) * sin(control_yaw_ - pose_yaw_);
  invGam_Det_ = term1 * term2;

  // Prevent division by zero with minimum threshold
  const double MIN_DENOMINATOR = 0.001;
  if (std::abs(invGam_Det_) < MIN_DENOMINATOR) {
    RCLCPP_WARN(
      this->get_logger(), "Small denominator detected in control matrix: %.6f", invGam_Det_);
    invGam_Det_ = std::copysign(MIN_DENOMINATOR, invGam_Det_);
  }

  // Calculate invGam components (condensing this section since it's internal math)
  double num1 = (params_.standstill_distance + params_.time_gap * odom_velocity_) * cos(pose_yaw_);
  double num2 = (params_.standstill_distance + params_.time_gap * odom_velocity_) * sin(pose_yaw_);
  double num3 =
    -params_.time_gap * sin(pose_yaw_) - params_.time_gap * sin(alpha_) * cos(control_yaw_);
  double num4 =
    params_.time_gap * cos(pose_yaw_) - params_.time_gap * sin(alpha_) * sin(control_yaw_);

  // Store invGam components for logging
  dbg_invGam_1_ = num1 / invGam_Det_;
  dbg_invGam_2_ = num2 / invGam_Det_;
  dbg_invGam_3_ = num3 / invGam_Det_;
  dbg_invGam_4_ = num4 / invGam_Det_;
  invGam_1_ = dbg_invGam_1_;
  invGam_2_ = dbg_invGam_2_;
  invGam_3_ = dbg_invGam_3_;
  invGam_4_ = dbg_invGam_4_;

  // Calculate inP terms (control inputs before transformation) and store components for logging
  dbg_inP1_pos_err_ = z_1_ * params_.kp;
  dbg_inP1_vel_err_ = cos(alpha_) * z_3_ + sin(alpha_) * z_4_;
  dbg_inP1_geom_vel_ = (1 - cos(alpha_)) * control_velocity_ * cos(control_yaw_) -
    sin(alpha_) * control_velocity_ * sin(control_yaw_);
  dbg_inP1_yaw_rate_ = cos(control_yaw_) * s_ * control_yaw_rate_;
  inP_1_ = dbg_inP1_pos_err_ + dbg_inP1_vel_err_ + dbg_inP1_geom_vel_ + dbg_inP1_yaw_rate_;

  dbg_inP2_pos_err_ = z_2_ * params_.kd;
  dbg_inP2_vel_err_ = sin(alpha_) * z_3_ + cos(alpha_) * z_4_;
  dbg_inP2_geom_vel_ = sin(alpha_) * control_velocity_ * cos(control_yaw_) +
    (1 - cos(alpha_)) * control_velocity_ * sin(control_yaw_);
  dbg_inP2_yaw_rate_ = sin(control_yaw_) * s_ * control_yaw_rate_;
  inP_2_ = dbg_inP2_pos_err_ + dbg_inP2_vel_err_ + dbg_inP2_geom_vel_ + dbg_inP2_yaw_rate_;

  // Calculate final control outputs
  a_ = invGam_1_ * inP_1_ + invGam_2_ * inP_2_;
  w_ = invGam_3_ * inP_1_ + invGam_4_ * inP_2_;

  RCLCPP_INFO(this->get_logger(), "Control Contributions:");
  RCLCPP_INFO(
    this->get_logger(), "  Position error contribution: %.2f m/s² (gain kp=%.2f)",
    z_1_ * params_.kp, params_.kp);
  RCLCPP_INFO(
    this->get_logger(), "  Velocity error contribution: %.2f m/s² (z3=%.2f, z4=%.2f)",
    cos(alpha_) * z_3_ + sin(alpha_) * z_4_, z_3_, z_4_);

  // Additional diagnostic logs for turns
  if (std::abs(control_curvature_) > 0.01) {
    RCLCPP_INFO(this->get_logger(), "Sharp Turn Detected:");
    RCLCPP_INFO(
      this->get_logger(), "  Leader yaw rate: %.2f rad/s, Leader velocity: %.2f m/s",
      control_yaw_rate_, control_velocity_);
    RCLCPP_INFO(
      this->get_logger(), "  Curvature effect on acceleration: %.2f m/s²",
      (cos(control_yaw_) * s_ * control_yaw_rate_));
  }

  last_time_ = rclcpp::Clock().now();

  // Update velocity
  v_ = last_velocity_ + a_ * dt_;

  // Check if we're reversing and log specifically about that
  if (v_ < 0 || a_ < -0.5) {
    RCLCPP_WARN(
      this->get_logger(), "Negative velocity/deceleration detected: v=%.2f m/s, a=%.2f m/s²", v_,
      a_);
    RCLCPP_WARN(
      this->get_logger(), "  Leader-follower velocity difference: %.2f m/s",
      control_velocity_ - odom_velocity_);
  }

  if (v_ > params_.max_velocity) {
    v_ = params_.max_velocity;
  }

  last_velocity_ = v_;

  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = v_;
  twist_msg.angular.z = w_;

  RCLCPP_INFO(this->get_logger(), "Final Controls:");
  RCLCPP_INFO(
    this->get_logger(), "  Acceleration: %.2f m/s², Angular velocity: %.2f rad/s", a_, w_);
  RCLCPP_INFO(this->get_logger(), "  Commanded velocity: %.2f m/s (dt=%.3fs)", v_, dt_);
  RCLCPP_INFO(this->get_logger(), "===========================");

  // Log data to file if enabled
  if (enable_data_logging_ && log_file_.is_open()) {
    // Get current timestamp
    auto now = this->get_clock()->now();
    double timestamp = now.seconds() + now.nanoseconds() * 1e-9;

    // Write all data to CSV, including new debug values
    log_file_ << std::fixed << std::setprecision(6) << timestamp << "," << cam_x_ << "," << cam_y_
              << "," << cam_velocity_ << "," << cam_acceleration_ << "," << cam_curvature_
              << ","  // Leader state
              << pose_x_ << "," << pose_y_ << "," << odom_velocity_ << "," << odom_acceleration_
              << ","  // Follower state
              << distance_term << "," << actual_distance << "," << (actual_distance - distance_term)
              << ","                                                       // Distances
              << z_1_ << "," << z_2_ << "," << z_3_ << "," << z_4_ << ","  // Error states
              << dbg_alpha_ << "," << dbg_s_ << ","                        // Debug: Geometry
              << dbg_invGam_1_ << "," << dbg_invGam_2_ << "," << dbg_invGam_3_ << ","
              << dbg_invGam_4_ << ","  // Debug: InvGamma
              << dbg_inP1_pos_err_ << "," << dbg_inP1_vel_err_ << "," << dbg_inP1_geom_vel_ << ","
              << dbg_inP1_yaw_rate_ << ","  // Debug: inP1 terms
              << dbg_inP2_pos_err_ << "," << dbg_inP2_vel_err_ << "," << dbg_inP2_geom_vel_ << ","
              << dbg_inP2_yaw_rate_ << ","     // Debug: inP2 terms
              << a_ << "," << v_ << "," << w_  // Final commands
              << std::endl;

    // Flush periodically to ensure data is written even if program crashes
    if (log_counter_++ % 10 == 0) {
      log_file_.flush();
    }
  }

  pub_cmd_vel->publish(twist_msg);
}

void CaccController::set_target_velocity(
  const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
  std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response)
{
  if (request->value == 1.0) {
    params_.target_velocity += 0.5;
  } else if (request->value == 0.0) {
    if (params_.target_velocity >= 1.0) {
      params_.target_velocity -= 0.5;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid value for target velocity");
  }

  RCLCPP_INFO(this->get_logger(), "Target velocity is now %f", params_.target_velocity);
  response->success = true;
}

void CaccController::set_extra_distance(
  const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
  std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Setting extra distance to %f", request->value);
  // if the request value is 1.0, increase the parameter by 0.5, if it is 0.0, decrease it by 0.5
  if (request->value == 1.0) {
    params_.extra_distance += 0.5;
  } else if (request->value == 0.0) {
    if (params_.extra_distance >= 0.5) {
      params_.extra_distance -= 0.5;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid value for extra distance");
  }

  RCLCPP_INFO(this->get_logger(), "Extra distance is now %f", params_.extra_distance);
  response->success = true;
}

void CaccController::set_standstill_distance(
  const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
  std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Setting standstill distance to %f", request->value);
  params_.standstill_distance = request->value;
  response->success = true;
}

void CaccController::set_time_gap(
  const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
  std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Setting time gap to %f", request->value);
  params_.time_gap = request->value;
  response->success = true;
}

void CaccController::set_cacc_enable(
  const std::shared_ptr<auna_msgs::srv::SetBool::Request> request,
  std::shared_ptr<auna_msgs::srv::SetBool::Response> response)
{
  if (!cacc_ready_) {
    RCLCPP_ERROR(this->get_logger(), "CACC controller not ready yet. Missing required messages.");
    response->success = false;
    return;
  }

  if (request->value) {
    RCLCPP_INFO(this->get_logger(), "Enabling CACC controller");
    last_cam_velocity_ = 0;
    last_odom_velocity_ = 0;
    last_velocity_ = 0;
    last_time_ = rclcpp::Clock().now();
    timer_->reset();
  } else {
    RCLCPP_INFO(this->get_logger(), "Disabling CACC controller");

    geometry_msgs::msg::Twist twist_msg;

    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;

    pub_cmd_vel->publish(twist_msg);

    timer_->cancel();
  }
  response->success = true;
}

// service server callback for auto_mode
void CaccController::set_auto_mode(
  const std::shared_ptr<auna_msgs::srv::SetBool::Request> request,
  std::shared_ptr<auna_msgs::srv::SetBool::Response> response)
{
  // Check if an actual CAM message is received - cannot be in auto mode if real CAMs are being
  // received
  if (last_cam_msg_ != nullptr) {
    RCLCPP_ERROR(this->get_logger(), "CACC controller is already running with real CAM messages");
    response->success = false;
    return;
  }

  // Check if both odom and pose are ready
  if (!auto_mode_ready_) {
    RCLCPP_ERROR(this->get_logger(), "Auto mode not ready yet. Missing required messages.");
    response->success = false;
    return;
  }

  if (request->value) {
    RCLCPP_INFO(this->get_logger(), "Enabling auto mode");
    auto_mode_ = true;
    timer_->reset();
    setup_timer_->cancel();
  } else {
    RCLCPP_INFO(this->get_logger(), "Disabling auto mode");

    geometry_msgs::msg::Twist twist_msg;

    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;

    pub_cmd_vel->publish(twist_msg);

    auto_mode_ = false;
    timer_->cancel();
  }
  response->success = true;
}

rcl_interfaces::msg::SetParametersResult CaccController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // print
    RCLCPP_INFO(this->get_logger(), "Parameter '%s' was changed.", name.c_str());

    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "standstill_distance") {
        params_.standstill_distance = parameter.as_double();
      } else if (name == "time_gap") {
        params_.time_gap = parameter.as_double();
      } else if (name == "kp") {
        params_.kp = parameter.as_double();
      } else if (name == "kd") {
        params_.kd = parameter.as_double();
      } else if (name == "max_velocity") {
        params_.max_velocity = parameter.as_double();
      } else if (name == "frequency") {
        params_.frequency = parameter.as_int();
      } else if (name == "use_waypoints") {
        params_.use_waypoints = parameter.as_bool();
      } else if (name == "waypoint_file") {
        params_.waypoint_file = parameter.as_string();
      } else if (name == "target_velocity") {
        params_.target_velocity = parameter.as_double();
      } else if (name == "curvature_lookahead") {
        params_.curvature_lookahead = parameter.as_double();
      } else if (name == "extra_distance") {
        params_.extra_distance = parameter.as_double();
      }
    }
  }

  result.successful = true;
  return result;
}
void CaccController::publish_waypoint_pose(
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & publisher,
  int waypoint_index)
{
  // Create and publish waypoint pose
  geometry_msgs::msg::PoseStamped waypoint_pose;
  waypoint_pose.header.stamp = this->get_clock()->now();
  waypoint_pose.header.frame_id = "map";
  waypoint_pose.pose.position.x = waypoints_x_[waypoint_index];
  waypoint_pose.pose.position.y = waypoints_y_[waypoint_index];
  waypoint_pose.pose.position.z = 0.0;

  // Convert yaw to quaternion
  tf2::Quaternion q_waypoint;
  q_waypoint.setRPY(0.0, 0.0, waypoints_yaw_[waypoint_index]);
  waypoint_pose.pose.orientation.x = q_waypoint.x();
  waypoint_pose.pose.orientation.y = q_waypoint.y();
  waypoint_pose.pose.orientation.z = q_waypoint.z();
  waypoint_pose.pose.orientation.w = q_waypoint.w();

  publisher->publish(waypoint_pose);
}
