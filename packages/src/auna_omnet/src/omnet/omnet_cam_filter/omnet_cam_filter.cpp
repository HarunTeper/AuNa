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

#include "auna_omnet/omnet_cam_filter.hpp"

OmnetCamFilter::OmnetCamFilter(int identifier)
: Node("omnet_cam_filter_node")
{
  cam_subscriber_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
    "cam", 2, [this](const etsi_its_cam_msgs::msg::CAM::SharedPtr msg) {cam_callback(msg);});
  cam_publisher_ = this->create_publisher<etsi_its_cam_msgs::msg::CAM>("cam_filtered", 2);
  service_ = this->create_service<auna_msgs::srv::Identifier>(
    "cam_filter_index", [this](
      const std::shared_ptr<auna_msgs::srv::Identifier::Request> request,
      std::shared_ptr<auna_msgs::srv::Identifier::Response> response) {
      filter_service_callback(request, response);
    });
  identifier_ = identifier;
}

// Set the identifier for filtering CAMs
void OmnetCamFilter::filter_service_callback(
  const std::shared_ptr<auna_msgs::srv::Identifier::Request> request,
  std::shared_ptr<auna_msgs::srv::Identifier::Response> response)
{
  identifier_ = request->identifier;
  response->success = true;
}

// Filter CAMs using the identifier
void OmnetCamFilter::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
{
  if (msg->header.station_id.value == (unsigned int)identifier_) {
    cam_publisher_->publish(*msg);
  }
}
