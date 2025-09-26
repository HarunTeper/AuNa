#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CaccWaypointPublisher : public rclcpp::Node
{
public:
  CaccWaypointPublisher() : Node("cacc_waypoint_publisher")
  {
    this->declare_parameter<std::string>("namespace", "");
    this->get_parameter<std::string>("namespace", namespace_);
    this->declare_parameter<std::string>("waypoint_file", "");
    this->get_parameter("waypoint_file", waypoint_file_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    std::string topic_name = namespace_.empty() ? "cacc/waypoints" : namespace_ + "/cacc/waypoints";
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(topic_name, qos);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      publish_waypoints();
      timer_->cancel();
    });
  }

private:
  void publish_waypoints()
  {
    std::ifstream file(waypoint_file_);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s", waypoint_file_.c_str());
      return;
    }

    auto pose_array_msg = std::make_unique<geometry_msgs::msg::PoseArray>();
    pose_array_msg->header.frame_id = "map";
    pose_array_msg->header.stamp = this->now();

    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      std::vector<double> values;
      while (std::getline(ss, value, ',')) {
        try {
          values.push_back(std::stod(value));
        } catch (const std::invalid_argument & ia) {
          continue;
        }
      }

      if (values.size() >= 2) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = values[0];
        pose.position.y = values[1];
        pose.position.z = 0.0;
        if (values.size() >= 3) {
          double yaw = values[2];
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          pose.orientation = tf2::toMsg(q);
        }
        pose_array_msg->poses.push_back(pose);
      }
    }

    // With yaw provided in the CSV, orientations are already set. No backfill needed.

    if (pose_array_msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "No waypoints were loaded from %s", waypoint_file_.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Publishing %zu waypoints from %s to %s", pose_array_msg->poses.size(),
      waypoint_file_.c_str(), publisher_->get_topic_name());
    publisher_->publish(std::move(pose_array_msg));
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string waypoint_file_;
  std::string namespace_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaccWaypointPublisher>());
  rclcpp::shutdown();
  return 0;
}