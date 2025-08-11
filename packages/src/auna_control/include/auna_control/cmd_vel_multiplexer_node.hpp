#pragma once

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

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

using SetBool = std_srvs::srv::SetBool;
using AckermannDriveStamped = ackermann_msgs::msg::AckermannDriveStamped;
using Twist = geometry_msgs::msg::Twist;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using StdBool = std_msgs::msg::Bool;
using Trigger = std_srvs::srv::Trigger;
using SetString = auna_msgs::srv::SetString;

namespace auna_control
{

struct InputSource
{
  std::string name;
  std::string topic;
  std::string type;
};

struct OutputTopic
{
  std::string name;
  std::string topic;
  std::string type;
};

class CmdVelMultiplexerNode : public rclcpp::Node
{
public:
  CmdVelMultiplexerNode();

private:
  void twist_callback(const TwistStamped::SharedPtr msg, const std::string & source_name);
  void twist_regular_callback(const Twist::SharedPtr msg, const std::string & source_name);
  void ackermann_callback(
    const AckermannDriveStamped::SharedPtr msg, const std::string & source_name);
  void toggleSourceCallback(
    const std::shared_ptr<SetString::Request> request,
    std::shared_ptr<SetString::Response> response);
  void setEstopCallback(
    const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response);
  void publishTimerCallback();
  AckermannDriveStamped createZeroAckermann();
  void getEstopStatusCallback(
    const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
  void getSourceStatusCallback(
    const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
  void getInputSourcesCallback(
    const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
  void debugStateCallback(
    const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  AckermannDriveStamped twist_to_ackermann(const TwistStamped::SharedPtr & twist_msg);
  AckermannDriveStamped twist_regular_to_ackermann(const Twist::SharedPtr & twist_msg);
  TwistStamped ackermann_to_twist(const AckermannDriveStamped & ackermann_msg);
  Twist ackermann_to_twist_regular(const AckermannDriveStamped & ackermann_msg);

  // Parameter parsing methods
  void parseInputSourcesFromYAML(const YAML::Node & config);
  void parseOutputTopicsFromYAML(const YAML::Node & config);
  void setupPublishers();
  void setupSubscribers();
  void updatePublishTimer();

  std::vector<InputSource> input_sources_;
  std::vector<OutputTopic> output_topics_;
  std::string current_source_;
  bool estop_active_;
  double publish_rate_;
  double wheelbase_;
  bool convert_yaw_to_steering_angle_;
  std::map<std::string, AckermannDriveStamped> last_received_msgs_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  std::map<std::string, rclcpp::Publisher<Twist>::SharedPtr> twist_regular_publishers_;
  std::map<std::string, rclcpp::Publisher<TwistStamped>::SharedPtr> twist_publishers_;
  std::map<std::string, rclcpp::Publisher<AckermannDriveStamped>::SharedPtr> ackermann_publishers_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> cmd_vel_subscribers_;
  rclcpp::Service<SetString>::SharedPtr set_source_service_;
  rclcpp::Service<SetBool>::SharedPtr set_estop_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Service<Trigger>::SharedPtr get_estop_status_service_;
  rclcpp::Service<Trigger>::SharedPtr get_source_status_service_;
  rclcpp::Service<Trigger>::SharedPtr get_input_sources_service_;
  rclcpp::Service<Trigger>::SharedPtr debug_state_service_;
};

}  // namespace auna_control