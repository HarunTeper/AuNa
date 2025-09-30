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


#include "auna_comm/cam_receiver.hpp"

#include <rclcpp/rclcpp.hpp>

#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <std_msgs/msg/string.hpp>

CamReceiver::CamReceiver()
: Node("cam_receiver")
{
  // Parameters
  this->declare_parameter<int>("filter_index", 0);

  // Create publisher with namespaced topic
  std::string output_topic = "cam_filtered";
  publisher_ = this->create_publisher<etsi_its_cam_msgs::msg::CAM>(output_topic, 10);

  subscription_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
    "/cam", 10, [this](const etsi_its_cam_msgs::msg::CAM::SharedPtr msg) {
      // Add debug logging for received CAMs
      RCLCPP_DEBUG(
        this->get_logger(), "Received CAM from station ID: %d, considering filter: %ld",
        msg->header.station_id.value, this->get_parameter("filter_index").as_int());
      // Log whether the message will be filtered or published
      if (msg->header.station_id.value == this->get_parameter("filter_index").as_int()) {
        RCLCPP_DEBUG(this->get_logger(), "Publishing CAM (station ID matches filter)");
      } else {
        RCLCPP_DEBUG(
          this->get_logger(), "Filtering out CAM (station ID %d doesn't match filter %ld)",
          msg->header.station_id.value, this->get_parameter("filter_index").as_int());
      }
      if (msg->header.station_id.value == this->get_parameter("filter_index").as_int()) {
        publisher_->publish(*msg);
      } else {
        RCLCPP_DEBUG(
          this->get_logger(), "Filtered CAM from station ID: %d", msg->header.station_id.value);
      }
    });
}
