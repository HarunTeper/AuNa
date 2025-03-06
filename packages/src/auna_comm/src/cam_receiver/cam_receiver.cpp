#include "auna_comm/cam_receiver.hpp"

#include <rclcpp/rclcpp.hpp>

#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <std_msgs/msg/string.hpp>

CamReceiver::CamReceiver() : Node("cam_receiver")
{
  // Parameters
  this->declare_parameter<int>("filter_index", 0);

  // Create publisher with namespaced topic
  std::string output_topic = "cam";
  publisher_ = this->create_publisher<etsi_its_cam_msgs::msg::CAM>(output_topic, 10);

  subscription_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
    "/cam", 10, [this](const etsi_its_cam_msgs::msg::CAM::SharedPtr msg) {
      // Add debug logging for received CAMs
      RCLCPP_INFO(
        this->get_logger(), "Received CAM from station ID: %d, considering filter: %ld",
        msg->header.station_id.value, this->get_parameter("filter_index").as_int());
      if (msg->header.station_id.value == this->get_parameter("filter_index").as_int()) {
        publisher_->publish(*msg);
      }
    });
}