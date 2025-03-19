#include "auna_cacc/cacc_controller.hpp"

#include <rclcpp/logging.hpp>

#include <etsi_its_msgs_utils/impl/cam/cam_getters_common.h>

#include <cmath>

CaccController::CaccController() : Node("cacc_controller")
{
  // Add startup logging
  RCLCPP_INFO(this->get_logger(), "Starting CACC controller, waiting for required messages");

  // Track if first messages have been received
  first_cam_received_ = false;
  first_odom_received_ = false;
  first_pose_received_ = false;

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
    "odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
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

  pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  RCLCPP_INFO(
    this->get_logger(), "Created cmd_vel publisher on topic: %s", pub_cmd_vel->get_topic_name());
  pub_x_lookahead_point_ = this->create_publisher<std_msgs::msg::Float64>("cacc/lookahead/x", 1);
  pub_y_lookahead_point_ = this->create_publisher<std_msgs::msg::Float64>("cacc/lookahead/y", 1);

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
  this->declare_parameter("kd", 3.0);
  this->declare_parameter("max_velocity", 1.0);
  this->declare_parameter("frequency", 50);
  this->declare_parameter("use_waypoints", false);
  this->declare_parameter("waypoint_file", "/home/$USER/waypoints.txt");
  this->declare_parameter("target_velocity", 1.0);
  this->declare_parameter("curvature_lookahead", 10);
  this->declare_parameter("extra_distance", 0.0);

  // use params_
  params_.standstill_distance = this->get_parameter("standstill_distance").as_double();
  params_.time_gap = this->get_parameter("time_gap").as_double();
  params_.kp = this->get_parameter("kp").as_double();
  params_.kd = this->get_parameter("kd").as_double();  // Increased slightly to reduce oversteering.
  params_.max_velocity = this->get_parameter("max_velocity").as_double();
  params_.frequency = this->get_parameter("frequency").as_int();
  params_.use_waypoints = this->get_parameter("use_waypoints").as_bool();
  params_.waypoint_file = this->get_parameter("waypoint_file").as_string();
  params_.target_velocity = this->get_parameter("target_velocity").as_double();
  params_.curvature_lookahead = this->get_parameter("curvature_lookahead").as_int();
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
    std::chrono::milliseconds(1000 / params_.frequency), [this]() { this->timer_callback(); });
  setup_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),  // Changed from 250ms to 1000ms for more frequent logs
    [this]() {
      this->setup_timer_callback();
      // Add debug log to show we're still waiting
      RCLCPP_INFO(
        this->get_logger(), "Waiting for messages - CAM: %s, Odom: %s, Pose: %s",
        (last_cam_msg_ != nullptr ? "received" : "waiting"),
        (last_odom_msg_ != nullptr ? "received" : "waiting"),
        (last_pose_msg_ != nullptr ? "received" : "waiting"));
    });
  timer_->cancel();

  if (params_.use_waypoints) {
    read_waypoints_from_csv();
  }

  dyn_params_handler_ = this->add_on_set_parameters_callback(
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
      return this->dynamicParametersCallback(parameters);
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
}

void CaccController::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "cam_callback - Generation Delta Time: %u",
    msg->cam.generation_delta_time.value);
  cam_x_ = msg->cam.cam_parameters.basic_container.reference_position.longitude.value / 10000000.0;
  cam_y_ = msg->cam.cam_parameters.basic_container.reference_position.latitude.value / 10000000.0;
  cam_velocity_ = msg->cam.cam_parameters.high_frequency_container
                    .basic_vehicle_container_high_frequency.speed.speed_value.value /
                  100.0;  // Convert from 0.01 m/s to m/s

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

    RCLCPP_INFO(this->get_logger(), "Generation Delta Time difference: %.4f", dt);

    dt = std::min(dt, 0.1);

    if (dt < 0.001) {
      cam_acceleration_ = 0.0;
    } else {
      cam_acceleration_ = (cam_velocity_ - last_cam_velocity_) / dt;
    }
  }
  last_cam_msg_ = msg;

  RCLCPP_INFO(
    this->get_logger(),
    "Received CAM - Speed: %.2f m/s, Accel: %.2f m/s², Yaw Rate: %.2f deg/s, Curvature: %.4f, "
    "Station ID: %d",
    cam_velocity_, cam_acceleration_, cam_yaw_rate_ * 180 / M_PI, cam_curvature_,
    msg->header.station_id.value);

  RCLCPP_INFO(
    this->get_logger(), "Heading: %.2f",
    msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading
        .heading_value.value /
      10.0);
  last_cam_velocity_ = cam_velocity_;

  cam_yaw_ = msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
               .heading.heading_value.value /
             10.0 * (M_PI / 180.0);  // Convert from 0.1 degrees to radians

  if (
    msg->cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.yaw_rate
        .yaw_rate_value.value != etsi_its_cam_msgs::msg::YawRateValue::UNAVAILABLE &&
    cam_velocity_ > 0.1) {
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
    odom_acceleration_ = odom_velocity_;
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
  if (auto_mode_) {
    double closest_distance_squared = std::numeric_limits<double>::max();
    int num_waypoints = waypoints_x_.size();

    int closest_waypoint_index;

    for (int i = 0; i < num_waypoints; ++i) {
      double dx = waypoints_x_[i] - pose_x_;
      double dy = waypoints_y_[i] - pose_y_;
      double distance_squared = dx * dx + dy * dy;

      if (distance_squared < closest_distance_squared) {
        closest_distance_squared = distance_squared;
        closest_waypoint_index = i;
      }
    }

    int target_waypoint_index_ = closest_waypoint_index;

    // Calculate target waypoint index (the waypoint that is closest to the vehicle and within the
    // time gap

    double target_distance =
      params_.standstill_distance + params_.time_gap * params_.target_velocity;

    for (int i = closest_waypoint_index; true; i = (i + 1) % num_waypoints) {
      double dx = waypoints_x_[i] - pose_x_;
      double dy = waypoints_y_[i] - pose_y_;
      double distance_squared = dx * dx + dy * dy;

      if (distance_squared > target_distance * target_distance) {
        // Stop the loop if the distance is increasing
        break;
      }

      target_waypoint_index_ = i;
    }
    cam_x_ = waypoints_x_[target_waypoint_index_];
    cam_y_ = waypoints_y_[target_waypoint_index_];

    // Calculate yaw difference between previous and next waypoints
    int curr_index = target_waypoint_index_;
    cam_yaw_ = waypoints_yaw_[curr_index];

    // Calculate cam_yaw_rate_ and cam_curvature_
    double current_yaw = waypoints_yaw_[curr_index];

    int next_index = (target_waypoint_index_ + params_.curvature_lookahead) % num_waypoints;
    double next_yaw = waypoints_yaw_[next_index];

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
    double dx = waypoints_x_[(curr_index + params_.curvature_lookahead) % num_waypoints] -
                waypoints_x_[curr_index];
    double dy = waypoints_y_[(curr_index + params_.curvature_lookahead) % num_waypoints] -
                waypoints_y_[curr_index];
    double distance = std::hypot(dx, dy);

    // calculate the required time by dividing distance through cam_velocity_
    double required_time = distance / params_.target_velocity;

    cam_yaw_rate_ = yaw_difference / required_time;

    // scale the cam_velocity_ between target_velocity and target_velocity/2, inverse proportional
    // to cam_yaw_rate_, whose values are between 0 and 1.25

    cam_velocity_ =
      params_.target_velocity - (params_.target_velocity / 2) * (cam_yaw_rate_ / 1.25);
    cam_acceleration_ = (cam_velocity_ - last_cam_velocity_) / dt_;
    last_cam_velocity_ = cam_velocity_;

    // cam curvature depending on auto_mode
    cam_curvature_ = cam_yaw_rate_ / params_.target_velocity;

  } else {
    double closest_distance_squared = std::numeric_limits<double>::max();
    int num_waypoints = waypoints_x_.size();

    int closest_waypoint_index;

    for (int i = 0; i < num_waypoints; ++i) {
      double dx = waypoints_x_[i] - cam_x_;
      double dy = waypoints_y_[i] - cam_y_;
      double distance_squared = dx * dx + dy * dy;

      if (distance_squared < closest_distance_squared) {
        closest_distance_squared = distance_squared;
        closest_waypoint_index = i;
      }
    }

    int target_waypoint_index_ = closest_waypoint_index;

    // Find the previous index at which the distance to the closest waypoint is at least
    // params_.extra_distance
    for (int i = closest_waypoint_index; true; i = (i - 1 + num_waypoints) % num_waypoints) {
      double dx = waypoints_x_[i] - waypoints_x_[closest_waypoint_index];
      double dy = waypoints_y_[i] - waypoints_y_[closest_waypoint_index];
      double distance_squared = dx * dx + dy * dy;

      if (distance_squared >= params_.extra_distance * params_.extra_distance) {
        // Stop the loop if the distance is increasing
        break;
      }

      target_waypoint_index_ = i;
    }

    cam_x_ = waypoints_x_[target_waypoint_index_];
    cam_y_ = waypoints_y_[target_waypoint_index_];
    cam_yaw_ = waypoints_yaw_[target_waypoint_index_];

    // Calculate cam_yaw_rate_ and cam_curvature_
    double current_yaw = waypoints_yaw_[target_waypoint_index_];

    int next_index = (target_waypoint_index_ + params_.curvature_lookahead) % num_waypoints;
    double next_yaw = waypoints_yaw_[next_index];

    // yaw difference modulo for the case that one is negative and the other positive
    double yaw_difference;
    if (next_yaw - current_yaw > M_PI) {
      yaw_difference = next_yaw - current_yaw - 2 * M_PI;
    } else if (next_yaw - current_yaw < -M_PI) {
      yaw_difference = next_yaw - current_yaw + 2 * M_PI;
    } else {
      yaw_difference = next_yaw - current_yaw;
    }

    // Calculate the required time to reach the n-th next waypoint
    double dx =
      waypoints_x_[(target_waypoint_index_ + params_.curvature_lookahead) % num_waypoints] -
      waypoints_x_[target_waypoint_index_];
    double dy =
      waypoints_y_[(target_waypoint_index_ + params_.curvature_lookahead) % num_waypoints] -
      waypoints_y_[target_waypoint_index_];
    double distance = std::hypot(dx, dy);

    // calculate the required time by dividing distance through cam_velocity_
    double required_time = distance / cam_velocity_;

    cam_yaw_rate_ = yaw_difference / required_time;

    // cam curvature depending on auto_mode
    if (cam_velocity_ == 0) {
      cam_curvature_ = 0;
    } else {
      cam_curvature_ = cam_yaw_rate_ / cam_velocity_;
    }
  }
}

void CaccController::timer_callback()
{
  if (params_.use_waypoints) {
    update_waypoint_following();
  }

  if (cam_curvature_ <= 0.01 && cam_curvature_ >= -0.01) {
    s_ = 0.5 * pow(params_.standstill_distance + params_.time_gap * odom_velocity_, 2) *
           cam_curvature_ -
         0.125 * pow(params_.standstill_distance + params_.time_gap * odom_velocity_, 4) *
           pow(cam_curvature_, 3);
  } else {
    s_ = (-1 + sqrt(
                 1 + pow(cam_curvature_, 2) *
                       pow(params_.standstill_distance + params_.time_gap * odom_velocity_, 2))) /
         cam_curvature_;
  }
  alpha_ = atan(cam_curvature_ * (params_.standstill_distance + params_.time_gap * odom_velocity_));

  x_lookahead_point_ = cam_x_ + s_ * sin(cam_yaw_);
  y_lookahead_point_ = cam_y_ - s_ * cos(cam_yaw_);

  std_msgs::msg::Float64 x_lookahead_point_msg;
  x_lookahead_point_msg.data = x_lookahead_point_;
  pub_x_lookahead_point_->publish(x_lookahead_point_msg);

  std_msgs::msg::Float64 y_lookahead_point_msg;
  y_lookahead_point_msg.data = y_lookahead_point_;
  pub_y_lookahead_point_->publish(y_lookahead_point_msg);

  z_1_ = cam_x_ - pose_x_ + s_ * sin(cam_yaw_) -
         (params_.standstill_distance + params_.time_gap * odom_velocity_) * cos(pose_yaw_);
  z_2_ = cam_y_ - pose_y_ - s_ * cos(cam_yaw_) -
         (params_.standstill_distance + params_.time_gap * odom_velocity_) * sin(pose_yaw_);
  z_3_ = cam_velocity_ * cos(cam_yaw_) - odom_velocity_ * cos(pose_yaw_ + alpha_);
  z_4_ = cam_velocity_ * sin(cam_yaw_) - odom_velocity_ * sin(pose_yaw_ + alpha_);

  // Calculate denominator with safety checks
  invGam_Det_ = (params_.standstill_distance + params_.time_gap * odom_velocity_) *
                (params_.time_gap - params_.time_gap * sin(alpha_) * sin(cam_yaw_ - pose_yaw_));

  // Prevent division by zero with minimum threshold
  const double MIN_DENOMINATOR = 0.001;
  if (std::abs(invGam_Det_) < MIN_DENOMINATOR) {
    invGam_Det_ = std::copysign(MIN_DENOMINATOR, invGam_Det_);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Low denominator detected: %.4f, using safety threshold", invGam_Det_);
  }

  RCLCPP_INFO(this->get_logger(), "  Before invGam calculations:");
  RCLCPP_INFO(
    this->get_logger(), "    params_.standstill_distance: %.4f", params_.standstill_distance);
  RCLCPP_INFO(this->get_logger(), "    params_.time_gap: %.4f", params_.time_gap);
  RCLCPP_INFO(this->get_logger(), "    odom_velocity_: %.4f", odom_velocity_);
  RCLCPP_INFO(this->get_logger(), "    pose_yaw_: %.4f", pose_yaw_);
  RCLCPP_INFO(this->get_logger(), "    alpha_: %.4f", alpha_);
  RCLCPP_INFO(this->get_logger(), "    cam_yaw_: %.4f", cam_yaw_);

  invGam_1_ = ((params_.standstill_distance + params_.time_gap * odom_velocity_) * cos(pose_yaw_)) /
              invGam_Det_;
  invGam_2_ = ((params_.standstill_distance + params_.time_gap * odom_velocity_) * sin(pose_yaw_)) /
              invGam_Det_;
  invGam_3_ =
    (-params_.time_gap * sin(pose_yaw_) - params_.time_gap * sin(alpha_) * cos(cam_yaw_)) /
    invGam_Det_;
  invGam_4_ = (params_.time_gap * cos(pose_yaw_) - params_.time_gap * sin(alpha_) * sin(cam_yaw_)) /
              invGam_Det_;

  inP_1_ = (z_1_ * params_.kp) + (cos(alpha_) * z_3_ + sin(alpha_) * z_4_) +
           ((1 - cos(alpha_)) * cam_velocity_ * cos(cam_yaw_) -
            sin(alpha_) * cam_velocity_ * sin(cam_yaw_)) +
           (cos(cam_yaw_) * s_ * cam_yaw_rate_);
  inP_2_ = (z_2_ * params_.kd) + (sin(alpha_) * z_3_ + cos(alpha_) * z_4_) +
           (sin(alpha_) * cam_velocity_ * cos(cam_yaw_) +
            (1 - cos(alpha_)) * cam_velocity_ * sin(cam_yaw_)) +
           (sin(cam_yaw_) * s_ * cam_yaw_rate_);

  a_ = invGam_1_ * inP_1_ + invGam_2_ * inP_2_;
  w_ = invGam_3_ * inP_1_ + invGam_4_ * inP_2_;

  last_time_ = rclcpp::Clock().now();

  v_ = last_velocity_ + a_ * dt_;

  if (v_ > params_.max_velocity) {
    v_ = params_.max_velocity;
  } else if (v_ < 0.01) {
    v_ = 0.0;
  }
  last_velocity_ = v_;

  geometry_msgs::msg::Twist twist_msg;

  twist_msg.linear.x = v_;
  twist_msg.angular.z = w_;

  // Detailed logging for debugging steering issues, throttled to 1Hz
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "--- CACC Timer Callback ---");
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "  cam_x: %.4f, cam_y: %.4f, cam_yaw: %.4f",
    cam_x_, cam_y_, cam_yaw_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "  pose_x: %.4f, pose_y: %.4f, pose_yaw: %.4f",
    pose_x_, pose_y_, pose_yaw_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "  cam_velocity: %.4f, odom_velocity: %.4f",
    cam_velocity_, odom_velocity_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "  cam_curvature: %.4f, cam_yaw_rate: %.4f",
    cam_curvature_, cam_yaw_rate_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "  standstill_distance: %.4f, time_gap: %.4f",
    params_.standstill_distance, params_.time_gap);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "  s: %.4f, alpha: %.4f", s_, alpha_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "  z1: %.4f, z2: %.4f, z3: %.4f, z4: %.4f", z_1_,
    z_2_, z_3_, z_4_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "  invGam_Det: %.4f", invGam_Det_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "  invGam_1: %.4f, invGam_2: %.4f, invGam_3: %.4f, invGam_4: %.4f", invGam_1_, invGam_2_,
    invGam_3_, invGam_4_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "  inP_1: %.4f, inP_2: %.4f", inP_1_, inP_2_);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "  a: %.4f, w: %.4f", a_, w_);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "  v: %.4f", v_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "  twist_msg.linear.x: %.4f, twist_msg.angular.z: %.4f", twist_msg.linear.x,
    twist_msg.angular.z);

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
        params_.curvature_lookahead = parameter.as_int();
      } else if (name == "extra_distance") {
        params_.extra_distance = parameter.as_double();
      }
    }
  }

  result.successful = true;
  return result;
}