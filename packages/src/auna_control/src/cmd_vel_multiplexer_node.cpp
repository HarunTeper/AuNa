#include "auna_control/cmd_vel_multiplexer_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "auna_msgs/srv/set_string.hpp"
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
using TwistStamped = geometry_msgs::msg::TwistStamped;
using StdBool = std_msgs::msg::Bool;
using Trigger = std_srvs::srv::Trigger;
using SetString = auna_msgs::srv::SetString;

namespace auna_control
{

CmdVelMultiplexerNode::CmdVelMultiplexerNode() : Node("cmd_multiplexer_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing CmdMultiplexerNode...");

  // Declare parameters for input sources
  this->declare_parameter("input_source_names", std::vector<std::string>{});
  this->declare_parameter("input_source_topics", std::vector<std::string>{});
  this->declare_parameter("input_source_types", std::vector<std::string>{});

  // Declare parameters for output topics
  this->declare_parameter("output_topic_names", std::vector<std::string>{});
  this->declare_parameter("output_topic_topics", std::vector<std::string>{});
  this->declare_parameter("output_topic_types", std::vector<std::string>{});

  this->declare_parameter("publish_rate", 20.0);

  // Parse input_sources
  std::vector<std::string> input_source_names =
    this->get_parameter("input_source_names").as_string_array();
  std::vector<std::string> input_source_topics =
    this->get_parameter("input_source_topics").as_string_array();
  std::vector<std::string> input_source_types =
    this->get_parameter("input_source_types").as_string_array();

  for (size_t i = 0; i < input_source_names.size(); ++i) {
    input_sources_.push_back(
      {input_source_names[i], input_source_topics[i], input_source_types[i]});
    RCLCPP_INFO(
      this->get_logger(), "Added input source: %s, topic: %s, type: %s",
      input_source_names[i].c_str(), input_source_topics[i].c_str(), input_source_types[i].c_str());
  }

  // Parse output_topics
  std::vector<std::string> output_topic_names =
    this->get_parameter("output_topic_names").as_string_array();
  std::vector<std::string> output_topic_topics =
    this->get_parameter("output_topic_topics").as_string_array();
  std::vector<std::string> output_topic_types =
    this->get_parameter("output_topic_types").as_string_array();

  for (size_t i = 0; i < output_topic_names.size(); ++i) {
    output_topics_.push_back(
      {output_topic_names[i], output_topic_topics[i], output_topic_types[i]});
    RCLCPP_INFO(
      this->get_logger(), "Added output topic: %s, topic: %s, type: %s",
      output_topic_names[i].c_str(), output_topic_topics[i].c_str(), output_topic_types[i].c_str());
  }

  double publish_rate = this->get_parameter("publish_rate").as_double();

  current_source_ = "OFF";
  estop_active_ = false;

  for (const auto & output_topic : output_topics_) {
    if (output_topic.type == "TwistStamped") {
      twist_publishers_[output_topic.name] =
        this->create_publisher<TwistStamped>(output_topic.topic, 10);
    } else if (output_topic.type == "AckermannDriveStamped") {
      ackermann_publishers_[output_topic.name] =
        this->create_publisher<AckermannDriveStamped>(output_topic.topic, 10);
    }
  }

  for (const auto & source : input_sources_) {
    if (source.type == "TwistStamped") {
      cmd_vel_subscribers_[source.name] = this->create_subscription<TwistStamped>(
        source.topic, 10, [this, source_name = source.name](const TwistStamped::SharedPtr msg) {
          this->twist_callback(msg, source_name);
        });
    } else if (source.type == "AckermannDriveStamped") {
      cmd_vel_subscribers_[source.name] = this->create_subscription<AckermannDriveStamped>(
        source.topic, 10,
        [this, source_name = source.name](const AckermannDriveStamped::SharedPtr msg) {
          this->ackermann_callback(msg, source_name);
        });
    }
    RCLCPP_INFO(
      this->get_logger(), "Subscribing to source '%s' on topic: %s", source.name.c_str(),
      source.topic.c_str());
    last_received_msgs_[source.name] = createZeroAckermann();
  }

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

  auto publish_period = std::chrono::duration<double>(1.0 / publish_rate);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
    std::bind(&CmdVelMultiplexerNode::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "CmdMultiplexerNode initialized.");
}

void CmdVelMultiplexerNode::twist_callback(
  const TwistStamped::SharedPtr msg, const std::string & source_name)
{
  last_received_msgs_[source_name] = twist_to_ackermann(msg);
}

void CmdVelMultiplexerNode::ackermann_callback(
  const AckermannDriveStamped::SharedPtr msg, const std::string & source_name)
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
  AckermannDriveStamped msg_to_publish;

  if (estop_active_) {
    msg_to_publish = createZeroAckermann();
  } else {
    bool has_message = last_received_msgs_.count(current_source_);
    if (current_source_ != "OFF" && has_message) {
      msg_to_publish = last_received_msgs_[current_source_];
    } else {
      msg_to_publish = createZeroAckermann();
    }
  }

  for (auto const & [name, publisher] : twist_publishers_) {
    publisher->publish(ackermann_to_twist(msg_to_publish));
  }
  for (auto const & [name, publisher] : ackermann_publishers_) {
    publisher->publish(msg_to_publish);
  }
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
  for (const auto & source : input_sources_) {
    sources += "," + source.name;
  }
  response->message = sources;
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

}  // namespace auna_control