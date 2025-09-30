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


#include "auna_control/cmd_vel_multiplexer_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "auna_msgs/srv/set_string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <algorithm>
#include <cctype>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using SetBool = std_srvs::srv::SetBool;
using AckermannDriveStamped = ackermann_msgs::msg::AckermannDriveStamped;
using Twist = geometry_msgs::msg::Twist;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using StdBool = std_msgs::msg::Bool;
using Trigger = std_srvs::srv::Trigger;
using SetString = auna_msgs::srv::SetString;

namespace auna_control
{

CmdVelMultiplexerNode::CmdVelMultiplexerNode()
: Node("cmd_vel_multiplexer_node", rclcpp::NodeOptions().allow_undeclared_parameters(true))
{
  RCLCPP_INFO(this->get_logger(), "Initializing CmdVelMultiplexerNode...");

  // Declare parameters with descriptors that allow runtime changes
  auto param_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  // Allow runtime changes

  this->declare_parameter("publish_rate", 20.0, param_descriptor);
  this->declare_parameter("wheelbase", 0.32, param_descriptor);
  this->declare_parameter("convert_yaw_to_steering_angle", false, param_descriptor);
  this->declare_parameter("topic_file", "", param_descriptor);
  this->declare_parameter("initial_source", "OFF", param_descriptor);

  std::string topic_file = this->get_parameter("topic_file").as_string();
  if (topic_file.empty()) {
    RCLCPP_ERROR(this->get_logger(), "topic_file parameter is not set.");
    RCLCPP_FATAL(
      this->get_logger(),
      "Shutting down CmdVelMultiplexerNode due to missing topic_file parameter.");
    rclcpp::shutdown();
    return;
  }

  YAML::Node config = YAML::LoadFile(topic_file);
  parseInputSourcesFromYAML(config);
  parseOutputTopicsFromYAML(config);

  double publish_rate = this->get_parameter("publish_rate").as_double();
  publish_rate_ = publish_rate;
  wheelbase_ = this->get_parameter("wheelbase").as_double();
  convert_yaw_to_steering_angle_ = this->get_parameter("convert_yaw_to_steering_angle").as_bool();

  // Determine initial source after parsing inputs
  std::string requested_initial_source = this->get_parameter("initial_source").as_string();
  current_source_ = "OFF";
  if (requested_initial_source != "OFF") {
    bool found = false;
    for (const auto & src : input_sources_) {
      if (src.name == requested_initial_source) {
        found = true;
        current_source_ = requested_initial_source;
        RCLCPP_INFO(
          this->get_logger(), "Initial cmd_vel source set to '%s'", current_source_.c_str());
        break;
      }
    }
    if (!found) {
      RCLCPP_WARN(
        this->get_logger(),
        "initial_source parameter '%s' not found in input sources. Starting with OFF.",
        requested_initial_source.c_str());
      RCLCPP_INFO(this->get_logger(), "Initial cmd_vel source set to OFF");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Initial cmd_vel source set to OFF");
  }
  estop_active_ = false;

  parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CmdVelMultiplexerNode::parameters_callback, this, std::placeholders::_1));

  setupPublishers();
  setupSubscribers();

  set_source_service_ = this->create_service<SetString>(
    "toggle_cmd_vel_source", std::bind(
                               &CmdVelMultiplexerNode::toggleSourceCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  set_estop_service_ = this->create_service<SetBool>(
    "/trigger_emergency_stop", std::bind(
                                 &CmdVelMultiplexerNode::setEstopCallback, this,
                                 std::placeholders::_1, std::placeholders::_2));

  get_estop_status_service_ = this->create_service<Trigger>(
    "/get_estop_status", std::bind(
                           &CmdVelMultiplexerNode::getEstopStatusCallback, this,
                           std::placeholders::_1, std::placeholders::_2));

  get_source_status_service_ = this->create_service<Trigger>(
    "get_source_status", std::bind(
                           &CmdVelMultiplexerNode::getSourceStatusCallback, this,
                           std::placeholders::_1, std::placeholders::_2));

  get_input_sources_service_ = this->create_service<Trigger>(
    "get_input_sources", std::bind(
                           &CmdVelMultiplexerNode::getInputSourcesCallback, this,
                           std::placeholders::_1, std::placeholders::_2));

  debug_state_service_ = this->create_service<Trigger>(
    "debug_state", std::bind(
                     &CmdVelMultiplexerNode::debugStateCallback, this, std::placeholders::_1,
                     std::placeholders::_2));

  updatePublishTimer();

  RCLCPP_INFO(this->get_logger(), "CmdVelMultiplexerNode initialized.");
}

void CmdVelMultiplexerNode::parseInputSourcesFromYAML(const YAML::Node & config)
{
  input_sources_.clear();
  const YAML::Node inputs = config["cmd_vel_multiplexer_node"]["inputs"];
  for (const auto & input : inputs) {
    std::string name = input["name"].as<std::string>();
    std::string topic = input["topic"].as<std::string>();
    std::string type = input["type"].as<std::string>();
    input_sources_.push_back({name, topic, type});
    RCLCPP_DEBUG(
      this->get_logger(), "Added input source: %s, topic: %s, type: %s", name.c_str(),
      topic.c_str(), type.c_str());
  }
}

void CmdVelMultiplexerNode::parseOutputTopicsFromYAML(const YAML::Node & config)
{
  output_topics_.clear();
  const YAML::Node outputs = config["cmd_vel_multiplexer_node"]["outputs"];
  for (const auto & output : outputs) {
    std::string name = output["name"].as<std::string>();
    std::string topic = output["topic"].as<std::string>();
    std::string type = output["type"].as<std::string>();
    output_topics_.push_back({name, topic, type});
    RCLCPP_DEBUG(
      this->get_logger(), "Added output topic: %s, topic: %s, type: %s", name.c_str(),
      topic.c_str(), type.c_str());
  }
}

void CmdVelMultiplexerNode::setupPublishers()
{
  for (const auto & output_topic : output_topics_) {
    if (output_topic.type == "Twist") {
      twist_regular_publishers_[output_topic.name] =
        this->create_publisher<Twist>(output_topic.topic, 10);
    } else if (output_topic.type == "TwistStamped") {
      twist_publishers_[output_topic.name] =
        this->create_publisher<TwistStamped>(output_topic.topic, 10);
    } else if (output_topic.type == "AckermannDriveStamped") {
      ackermann_publishers_[output_topic.name] =
        this->create_publisher<AckermannDriveStamped>(output_topic.topic, 10);
    }
  }
}

void CmdVelMultiplexerNode::setupSubscribers()
{
  for (const auto & source : input_sources_) {
    if (source.type == "TwistStamped") {
      cmd_vel_subscribers_[source.name] = this->create_subscription<TwistStamped>(
        source.topic, 10, [this, source_name = source.name](const TwistStamped::SharedPtr msg) {
          this->twist_callback(msg, source_name);
        });
    } else if (source.type == "Twist") {
      cmd_vel_subscribers_[source.name] = this->create_subscription<Twist>(
        source.topic, 10, [this, source_name = source.name](const Twist::SharedPtr msg) {
          this->twist_regular_callback(msg, source_name);
        });
    } else if (source.type == "AckermannDriveStamped") {
      cmd_vel_subscribers_[source.name] = this->create_subscription<AckermannDriveStamped>(
        source.topic, 10,
        [this, source_name = source.name](const AckermannDriveStamped::SharedPtr msg) {
          this->ackermann_callback(msg, source_name);
        });
    }
    RCLCPP_DEBUG(
      this->get_logger(), "Subscribing to source '%s' on topic: %s", source.name.c_str(),
      source.topic.c_str());
    last_received_msgs_[source.name] = createZeroAckermann();
  }
}

void CmdVelMultiplexerNode::twist_regular_callback(
  const Twist::SharedPtr msg, const std::string & source_name)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received Twist (regular) message from source: %s, linear.x: %f, angular.z: %f",
    source_name.c_str(), msg->linear.x, msg->angular.z);
  last_received_msgs_[source_name] = twist_regular_to_ackermann(msg);
  RCLCPP_DEBUG(
    this->get_logger(), "Stored in last_received_msgs_[%s]: speed=%f, steering_angle=%f",
    source_name.c_str(), last_received_msgs_[source_name].drive.speed,
    last_received_msgs_[source_name].drive.steering_angle);
}

AckermannDriveStamped CmdVelMultiplexerNode::twist_regular_to_ackermann(
  const Twist::SharedPtr & twist_msg)
{
  AckermannDriveStamped ackermann_msg;
  ackermann_msg.header.stamp = this->now();
  ackermann_msg.drive.speed = twist_msg->linear.x;
  ackermann_msg.drive.steering_angle = twist_msg->angular.z;
  return ackermann_msg;
}

void CmdVelMultiplexerNode::twist_callback(
  const TwistStamped::SharedPtr msg, const std::string & source_name)
{
  RCLCPP_DEBUG(
    this->get_logger(), "Received Twist message from source: %s, linear.x: %f, angular.z: %f",
    source_name.c_str(), msg->twist.linear.x, msg->twist.angular.z);
  last_received_msgs_[source_name] = twist_to_ackermann(msg);
  RCLCPP_DEBUG(
    this->get_logger(), "Stored in last_received_msgs_[%s]: speed=%f, steering_angle=%f",
    source_name.c_str(), last_received_msgs_[source_name].drive.speed,
    last_received_msgs_[source_name].drive.steering_angle);
}

void CmdVelMultiplexerNode::ackermann_callback(
  const AckermannDriveStamped::SharedPtr msg, const std::string & source_name)
{
  RCLCPP_DEBUG(
    this->get_logger(), "Received Ackermann message from source: %s, speed: %f, steering_angle: %f",
    source_name.c_str(), msg->drive.speed, msg->drive.steering_angle);
  last_received_msgs_[source_name] = *msg;
}

void CmdVelMultiplexerNode::toggleSourceCallback(
  const std::shared_ptr<SetString::Request> request, std::shared_ptr<SetString::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "toggleSourceCallback called");
  std::string requested_source = request->data;
  if (requested_source == "OFF") {
    current_source_ = "OFF";
    response->success = true;
    response->message = "Velocity source turned OFF";
  } else {
    bool found = false;
    for (const auto & source : input_sources_) {
      if (source.name == requested_source) {
        found = true;
        break;
      }
    }
    if (found) {
      current_source_ = requested_source;
      response->success = true;
      response->message = "Velocity source set to: " + current_source_;
    } else {
      response->success = false;
      response->message = "Requested source '" + requested_source + "' not in input_sources";
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "%s", response->message.c_str());
}

void CmdVelMultiplexerNode::setEstopCallback(
  const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "setEstopCallback called");
  estop_active_ = request->data;
  response->success = true;
  response->message = estop_active_ ? "Emergency stop ACTIVATED" : "Emergency stop DEACTIVATED";
  if (estop_active_) {
    RCLCPP_WARN(this->get_logger(), "Namespaced E-Stop Service: %s", response->message.c_str());
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Namespaced E-Stop Service: %s", response->message.c_str());
  }
}

void CmdVelMultiplexerNode::publishTimerCallback()
{
  AckermannDriveStamped msg_to_publish;

  RCLCPP_DEBUG(
    this->get_logger(), "publishTimerCallback: current_source_='%s', estop_active_=%s",
    current_source_.c_str(), estop_active_ ? "true" : "false");

  if (estop_active_) {
    msg_to_publish = createZeroAckermann();
    RCLCPP_DEBUG(this->get_logger(), "E-stop active, publishing zero message");
  } else {
    bool has_message = last_received_msgs_.count(current_source_);
    RCLCPP_DEBUG(
      this->get_logger(), "has_message for source '%s': %s", current_source_.c_str(),
      has_message ? "true" : "false");

    if (current_source_ != "OFF" && has_message) {
      msg_to_publish = last_received_msgs_[current_source_];
      RCLCPP_DEBUG(
        this->get_logger(), "Using message from source '%s': speed=%f, steering_angle=%f",
        current_source_.c_str(), msg_to_publish.drive.speed, msg_to_publish.drive.steering_angle);
    } else {
      msg_to_publish = createZeroAckermann();
      RCLCPP_DEBUG(this->get_logger(), "Source is OFF or no message available, publishing zero");
    }
  }

  if (convert_yaw_to_steering_angle_) {
    double linear_velocity = msg_to_publish.drive.speed;
    double yaw_rate = msg_to_publish.drive.steering_angle;
    if (std::abs(linear_velocity) > 1e-2) {
      msg_to_publish.drive.steering_angle = std::atan(wheelbase_ * yaw_rate / linear_velocity);
    } else {
      msg_to_publish.drive.steering_angle = 0.0;
    }
  }

  for (auto const & [name, publisher] : twist_regular_publishers_) {
    publisher->publish(ackermann_to_twist_regular(msg_to_publish));
  }
  for (auto const & [name, publisher] : twist_publishers_) {
    publisher->publish(ackermann_to_twist(msg_to_publish));
  }
  for (auto const & [name, publisher] : ackermann_publishers_) {
    publisher->publish(msg_to_publish);
  }
  RCLCPP_DEBUG(
    this->get_logger(), "Publishing message with speed: %f, steering_angle: %f (source: %s)",
    msg_to_publish.drive.speed, msg_to_publish.drive.steering_angle, current_source_.c_str());
}

void CmdVelMultiplexerNode::getEstopStatusCallback(
  const std::shared_ptr<Trigger::Request> /*request*/, std::shared_ptr<Trigger::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "getEstopStatusCallback called");
  response->success = true;
  response->message = estop_active_ ? "Emergency stop is ACTIVE" : "Emergency stop is INACTIVE";
}

void CmdVelMultiplexerNode::getSourceStatusCallback(
  const std::shared_ptr<Trigger::Request> /*request*/, std::shared_ptr<Trigger::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "getSourceStatusCallback called");
  response->success = true;
  response->message = "Current source: " + current_source_;
}

void CmdVelMultiplexerNode::getInputSourcesCallback(
  const std::shared_ptr<Trigger::Request> /*request*/, std::shared_ptr<Trigger::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "getInputSourcesCallback called");
  response->success = true;
  std::string sources = "OFF";
  for (const auto & source : input_sources_) {
    sources += "," + source.name;
  }
  response->message = sources;
}

void CmdVelMultiplexerNode::debugStateCallback(
  const std::shared_ptr<Trigger::Request> /*request*/, std::shared_ptr<Trigger::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "debugStateCallback called");
  response->success = true;

  std::string debug_info = "Debug State:\n";
  debug_info += "current_source_: " + current_source_ + "\n";
  debug_info += "estop_active_: " + std::string(estop_active_ ? "true" : "false") + "\n";
  debug_info += "last_received_msgs_ contents:\n";

  for (const auto & [source_name, ackermann_msg] : last_received_msgs_) {
    debug_info += "  " + source_name + ": speed=" + std::to_string(ackermann_msg.drive.speed) +
                  ", steering_angle=" + std::to_string(ackermann_msg.drive.steering_angle) + "\n";
  }

  response->message = debug_info;
}

rcl_interfaces::msg::SetParametersResult CmdVelMultiplexerNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "publish_rate") {
      publish_rate_ = parameter.as_double();
      RCLCPP_DEBUG(this->get_logger(), "Publish rate parameter set to: %f", publish_rate_);
      updatePublishTimer();
    } else if (parameter.get_name() == "wheelbase") {
      wheelbase_ = parameter.as_double();
      RCLCPP_DEBUG(this->get_logger(), "Wheelbase parameter set to: %f", wheelbase_);
    } else if (parameter.get_name() == "convert_yaw_to_steering_angle") {
      convert_yaw_to_steering_angle_ = parameter.as_bool();
      RCLCPP_DEBUG(
        this->get_logger(), "convert_yaw_to_steering_angle parameter set to: %s",
        convert_yaw_to_steering_angle_ ? "true" : "false");
    }
  }
  return result;
}

AckermannDriveStamped CmdVelMultiplexerNode::createZeroAckermann()
{
  AckermannDriveStamped msg;
  msg.drive.speed = 0.0;
  msg.drive.steering_angle = 0.0;
  return msg;
}

AckermannDriveStamped CmdVelMultiplexerNode::twist_to_ackermann(
  const TwistStamped::SharedPtr & twist_msg)
{
  AckermannDriveStamped ackermann_msg;
  ackermann_msg.header = twist_msg->header;
  ackermann_msg.drive.speed = twist_msg->twist.linear.x;
  ackermann_msg.drive.steering_angle = twist_msg->twist.angular.z;
  return ackermann_msg;
}

TwistStamped CmdVelMultiplexerNode::ackermann_to_twist(const AckermannDriveStamped & ackermann_msg)
{
  TwistStamped twist_msg;
  twist_msg.header = ackermann_msg.header;
  twist_msg.twist.linear.x = ackermann_msg.drive.speed;
  twist_msg.twist.angular.z = ackermann_msg.drive.steering_angle;
  return twist_msg;
}

Twist CmdVelMultiplexerNode::ackermann_to_twist_regular(const AckermannDriveStamped & ackermann_msg)
{
  Twist twist_msg;
  twist_msg.linear.x = ackermann_msg.drive.speed;
  twist_msg.angular.z = ackermann_msg.drive.steering_angle;
  return twist_msg;
}

void CmdVelMultiplexerNode::updatePublishTimer()
{
  // Cancel the existing timer if it exists
  if (publish_timer_) {
    publish_timer_->cancel();
  }

  // Create a new timer with the updated publish rate
  auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
    std::bind(&CmdVelMultiplexerNode::publishTimerCallback, this));

  RCLCPP_DEBUG(this->get_logger(), "Updated publish timer with rate: %f Hz", publish_rate_);
}

}  // namespace auna_control