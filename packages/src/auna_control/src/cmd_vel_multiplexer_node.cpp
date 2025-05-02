#include "rclcpp/rclcpp.hpp"

#include "auna_msgs/srv/set_string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <algorithm>
#include <cctype>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using SetString = auna_msgs::srv::SetString;
using SetBool = std_srvs::srv::SetBool;
using Twist = geometry_msgs::msg::Twist;

class CmdVelMultiplexerNode : public rclcpp::Node
{
public:
  CmdVelMultiplexerNode() : Node("cmd_vel_multiplexer_node")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing CmdVelMultiplexerNode...");

    this->declare_parameter<std::string>("output_topic", "cmd_vel");
    this->declare_parameter("input_sources", std::vector<std::string>{});
    this->declare_parameter<std::string>("default_source", "OFF");
    this->declare_parameter<double>("publish_rate", 20.0);

    output_topic_ = this->get_parameter("output_topic").as_string();
    input_sources_ = this->get_parameter("input_sources").as_string_array();
    current_source_ = this->get_parameter("default_source").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    RCLCPP_INFO(
      this->get_logger(), "Output topic: '%s', Default source: '%s', Publish rate: %.1f Hz",
      output_topic_.c_str(), current_source_.c_str(), publish_rate);
    RCLCPP_INFO(this->get_logger(), "Number of input sources: %zu", input_sources_.size());

    estop_active_ = false;

    cmd_vel_publisher_ = this->create_publisher<Twist>(output_topic_, 10);

    for (const auto & source_name : input_sources_) {
      std::string lower_source_name = source_name;
      std::transform(
        lower_source_name.begin(), lower_source_name.end(), lower_source_name.begin(),
        [](unsigned char c) { return std::tolower(c); });
      std::string input_topic = "cmd_vel_" + lower_source_name;
      auto callback = [this, source_name](const Twist::SharedPtr msg) {
        this->cmdVelCallback(source_name, msg);
      };
      cmd_vel_subscribers_[source_name] =
        this->create_subscription<Twist>(input_topic, 10, callback);
      RCLCPP_INFO(
        this->get_logger(), "Subscribing to source '%s' on topic: %s", source_name.c_str(),
        input_topic.c_str());
      last_received_msgs_[source_name] = createZeroTwist();
    }

    set_source_service_ = this->create_service<SetString>(
      "set_cmd_vel_source", std::bind(
                              &CmdVelMultiplexerNode::setSourceCallback, this,
                              std::placeholders::_1, std::placeholders::_2));

    set_estop_service_ = this->create_service<SetBool>(
      "trigger_emergency_stop", std::bind(
                                  &CmdVelMultiplexerNode::setEstopCallback, this,
                                  std::placeholders::_1, std::placeholders::_2));

    auto publish_period = std::chrono::duration<double>(1.0 / publish_rate);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&CmdVelMultiplexerNode::publishTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "CmdVelMultiplexerNode initialized.");
  }

private:
  void cmdVelCallback(const std::string & source_name, const Twist::SharedPtr msg)
  {
    last_received_msgs_[source_name] = *msg;
  }

  void setSourceCallback(
    const std::shared_ptr<SetString::Request> request,
    std::shared_ptr<SetString::Response> response)
  {
    std::string requested_source = request->data;
    RCLCPP_DEBUG(this->get_logger(), "Service call to set source to: %s", requested_source.c_str());

    bool valid_source = (requested_source == "OFF");
    if (!valid_source) {
      for (const auto & known_source : input_sources_) {
        if (requested_source == known_source) {
          valid_source = true;
          break;
        }
      }
    }

    if (valid_source) {
      current_source_ = requested_source;
      response->success = true;
      response->message = "Source set to " + current_source_;
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
      response->success = false;
      response->message = "Invalid source requested: " + requested_source;
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    }
  }

  void setEstopCallback(
    const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response)
  {
    estop_active_ = request->data;
    response->success = true;
    response->message = estop_active_ ? "Emergency stop ACTIVATED" : "Emergency stop DEACTIVATED";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
  }

  void publishTimerCallback()
  {
    RCLCPP_DEBUG(
      this->get_logger(), "Publish timer: Current source='%s', E-stop=%s", current_source_.c_str(),
      estop_active_ ? "true" : "false");
    Twist msg_to_publish;

    if (estop_active_) {
      msg_to_publish = createZeroTwist();
    } else {
      bool has_message = last_received_msgs_.count(current_source_);
      if (current_source_ != "OFF" && has_message) {
        msg_to_publish = last_received_msgs_[current_source_];
        RCLCPP_DEBUG(
          this->get_logger(), "Publishing message from '%s'. Linear.x=%.2f",
          current_source_.c_str(), msg_to_publish.linear.x);
      } else {
        msg_to_publish = createZeroTwist();
        RCLCPP_DEBUG(this->get_logger(), "Publishing zero twist (Source OFF or no message).");
      }
    }
    cmd_vel_publisher_->publish(msg_to_publish);
  }

  Twist createZeroTwist()
  {
    Twist zero_twist;
    zero_twist.linear.x = 0.0;
    zero_twist.linear.y = 0.0;
    zero_twist.linear.z = 0.0;
    zero_twist.angular.x = 0.0;
    zero_twist.angular.y = 0.0;
    zero_twist.angular.z = 0.0;
    return zero_twist;
  }

  std::string output_topic_;
  std::vector<std::string> input_sources_;

  std::string current_source_;
  bool estop_active_;
  std::map<std::string, Twist> last_received_msgs_;

  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
  std::map<std::string, rclcpp::Subscription<Twist>::SharedPtr> cmd_vel_subscribers_;
  rclcpp::Service<SetString>::SharedPtr set_source_service_;
  rclcpp::Service<SetBool>::SharedPtr set_estop_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelMultiplexerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}