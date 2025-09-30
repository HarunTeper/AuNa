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

#ifndef AUNA_OMNET__OMNET_CAM_FILTER_HPP_
#define AUNA_OMNET__OMNET_CAM_FILTER_HPP_

#include <memory>

#include "auna_msgs/srv/identifier.hpp"
#include "etsi_its_cam_msgs/msg/cam.hpp"
#include "rclcpp/rclcpp.hpp"

class OmnetCamFilter : public rclcpp::Node
{
public:
  explicit OmnetCamFilter(int identifier);

private:
  void cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg);
  void filter_service_callback(
    const std::shared_ptr<auna_msgs::srv::Identifier::Request> request,
    std::shared_ptr<auna_msgs::srv::Identifier::Response> response);
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_publisher_;
  rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_subscriber_;
  rclcpp::Service<auna_msgs::srv::Identifier>::SharedPtr service_;

  int identifier_ = 0;
};

#endif  // AUNA_OMNET__OMNET_CAM_FILTER_HPP_
