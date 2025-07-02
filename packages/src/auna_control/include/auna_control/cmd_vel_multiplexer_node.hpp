#pragma once

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

using SetBool = std_srvs::srv::SetBool;
using AckermannDriveStamped = ackermann_msgs::msg::AckermannDriveStamped;
using StdBool = std_msgs::msg::Bool;
using Trigger = std_srvs::srv::Trigger;
using SetString = auna_msgs::srv::SetString;

namespace auna_control
{

class CmdVelMultiplexerNode : public rclcpp::Node
{
public:
  CmdVelMultiplexerNode();

private:
  void cmdVelCallback(const std::string & source_name, const AckermannDriveStamped::SharedPtr msg);
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

  std::string output_topic_;
  std::vector<std::string> input_sources_;
  std::string current_source_;
  bool estop_active_;
  std::map<std::string, AckermannDriveStamped> last_received_msgs_;

  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr cmd_vel_publisher_;
  std::map<std::string, rclcpp::Subscription<AckermannDriveStamped>::SharedPtr>
    cmd_vel_subscribers_;
  rclcpp::Service<SetString>::SharedPtr set_source_service_;
  rclcpp::Service<SetBool>::SharedPtr set_estop_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Service<Trigger>::SharedPtr get_estop_status_service_;
  rclcpp::Service<Trigger>::SharedPtr get_source_status_service_;
  rclcpp::Service<Trigger>::SharedPtr get_input_sources_service_;
};

}  // namespace auna_control