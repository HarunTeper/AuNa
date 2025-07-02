#include "auna_control/cmd_vel_multiplexer_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "auna_msgs/srv/set_string.hpp"
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
using StdBool = std_msgs::msg::Bool;
using Trigger = std_srvs::srv::Trigger;
using SetString = auna_msgs::srv::SetString;

namespace auna_control
{

CmdVelMultiplexerNode::CmdVelMultiplexerNode() : Node("cmd_multiplexer_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing CmdMultiplexerNode...");

  this->declare_parameter<std::string>("output_topic", "cmd_vel");
  this->declare_parameter("input_sources", std::vector<std::string>{});
  this->declare_parameter<double>("publish_rate", 20.0);

  output_topic_ = this->get_parameter("output_topic").as_string();
  input_sources_ = this->get_parameter("input_sources").as_string_array();
  current_source_ = "OFF";
  double publish_rate = this->get_parameter("publish_rate").as_double();

  RCLCPP_INFO(
    this->get_logger(), "Output topic: '%s', Publish rate: %.1f Hz", output_topic_.c_str(),
    publish_rate);
  RCLCPP_INFO(this->get_logger(), "Number of input sources: %zu", input_sources_.size());

  estop_active_ = false;

  cmd_vel_publisher_ = this->create_publisher<AckermannDriveStamped>(output_topic_, 10);

  for (const auto & source_name : input_sources_) {
    std::string lower_source_name = source_name;
    std::transform(
      lower_source_name.begin(), lower_source_name.end(), lower_source_name.begin(),
      [](unsigned char c) { return std::tolower(c); });
    std::string input_topic = "cmd_vel" + lower_source_name;
    auto callback = [this, source_name](const AckermannDriveStamped::SharedPtr msg) {
      this->cmdVelCallback(source_name, msg);
    };
    cmd_vel_subscribers_[source_name] =
      this->create_subscription<AckermannDriveStamped>(input_topic, 10, callback);
    RCLCPP_INFO(
      this->get_logger(), "Subscribing to source '%s' on topic: %s", source_name.c_str(),
      input_topic.c_str());
    last_received_msgs_[source_name] = createZeroAckermann();
  }

  set_source_service_ = this->create_service<SetString>(
    "toggle_cmd_vel_source", std::bind(
                               &CmdVelMultiplexerNode::toggleSourceCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  set_estop_service_ = this->create_service<SetBool>(
    "/trigger_emergency_stop", std::bind(
                                 &CmdVelMultiplexerNode::setEstopCallback, this,
                                 std::placeholders::_1, std::placeholders::_2));

  // New: Service to get estop status
  get_estop_status_service_ = this->create_service<Trigger>(
    "/get_estop_status", std::bind(
                           &CmdVelMultiplexerNode::getEstopStatusCallback, this,
                           std::placeholders::_1, std::placeholders::_2));

  // New: Service to get current source
  get_source_status_service_ = this->create_service<Trigger>(
    "get_source_status", std::bind(
                           &CmdVelMultiplexerNode::getSourceStatusCallback, this,
                           std::placeholders::_1, std::placeholders::_2));

  // Add a new service to get the list of input sources
  get_input_sources_service_ = this->create_service<Trigger>(
    "get_input_sources", std::bind(
                           &CmdVelMultiplexerNode::getInputSourcesCallback, this,
                           std::placeholders::_1, std::placeholders::_2));

  auto publish_period = std::chrono::duration<double>(1.0 / publish_rate);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
    std::bind(&CmdVelMultiplexerNode::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "CmdMultiplexerNode initialized.");
}

void CmdVelMultiplexerNode::cmdVelCallback(
  const std::string & source_name, const AckermannDriveStamped::SharedPtr msg)
{
  last_received_msgs_[source_name] = *msg;
}

void CmdVelMultiplexerNode::toggleSourceCallback(
  const std::shared_ptr<SetString::Request> request, std::shared_ptr<SetString::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "toggleSourceCallback called");
  std::string requested_source = request->data;
  if (requested_source == "OFF") {
    current_source_ = "OFF";
    response->success = true;
    response->message = "Velocity source turned OFF";
  } else if (
    std::find(input_sources_.begin(), input_sources_.end(), requested_source) !=
    input_sources_.end()) {
    current_source_ = requested_source;
    response->success = true;
    response->message = "Velocity source set to: " + current_source_;
  } else {
    response->success = false;
    response->message = "Requested source '" + requested_source + "' not in input_sources";
  }
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void CmdVelMultiplexerNode::setEstopCallback(
  const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "setEstopCallback called");
  estop_active_ = request->data;
  response->success = true;
  response->message = estop_active_ ? "Emergency stop ACTIVATED" : "Emergency stop DEACTIVATED";
  if (estop_active_) {
    RCLCPP_WARN(this->get_logger(), "Namespaced E-Stop Service: %s", response->message.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Namespaced E-Stop Service: %s", response->message.c_str());
  }
}

void CmdVelMultiplexerNode::publishTimerCallback()
{
  RCLCPP_DEBUG(
    this->get_logger(), "Publish timer: Current source='%s', E-stop=%s", current_source_.c_str(),
    estop_active_ ? "true" : "false");
  AckermannDriveStamped msg_to_publish;

  if (estop_active_) {
    msg_to_publish = createZeroAckermann();
  } else {
    bool has_message = last_received_msgs_.count(current_source_);
    if (current_source_ != "OFF" && has_message) {
      msg_to_publish = last_received_msgs_[current_source_];
      RCLCPP_DEBUG(
        this->get_logger(), "Publishing message from '%s'. Speed=%.2f", current_source_.c_str(),
        msg_to_publish.drive.speed);
    } else {
      msg_to_publish = createZeroAckermann();
      RCLCPP_DEBUG(this->get_logger(), "Publishing zero ackermann (Source OFF or no message).");
    }
  }
  cmd_vel_publisher_->publish(msg_to_publish);
}

void CmdVelMultiplexerNode::getEstopStatusCallback(
  const std::shared_ptr<Trigger::Request> /*request*/, std::shared_ptr<Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "getEstopStatusCallback called");
  response->success = true;
  response->message = estop_active_ ? "Emergency stop is ACTIVE" : "Emergency stop is INACTIVE";
}

void CmdVelMultiplexerNode::getSourceStatusCallback(
  const std::shared_ptr<Trigger::Request> /*request*/, std::shared_ptr<Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "getSourceStatusCallback called");
  response->success = true;
  response->message = "Current source: " + current_source_;
}

void CmdVelMultiplexerNode::getInputSourcesCallback(
  const std::shared_ptr<Trigger::Request> /*request*/, std::shared_ptr<Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "getInputSourcesCallback called");
  response->success = true;
  std::string sources = "OFF";
  for (const auto & src : input_sources_) {
    sources += "," + src;
  }
  response->message = sources;
}

AckermannDriveStamped CmdVelMultiplexerNode::createZeroAckermann()
{
  return AckermannDriveStamped{};
}

}  // namespace auna_control