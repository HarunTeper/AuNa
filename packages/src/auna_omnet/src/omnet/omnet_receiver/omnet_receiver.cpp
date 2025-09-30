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

#include "auna_omnet/omnet_receiver.hpp"

#include <cmath>  // Required for M_PI, floor, fmod (though floor might not be needed with simpler heading)
#include <string>  // Potentially for std::to_string if direct int to string for robot_name is needed later

// Create a publisher and subscriber
OmnetReceiver::OmnetReceiver() : Node("omnet_receiver_node") {
  omnet_subscriber_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
      "cam_in", 2, [this](const etsi_its_cam_msgs::msg::CAM::SharedPtr msg) {
        cam_callback(msg);
      });
  // Publisher type corrected in the header, this initialization should now
  // match.
  cam_publisher_ =
      this->create_publisher<etsi_its_cam_msgs::msg::CAM>("cam", 2);
}

// Receive the CAM from Artery and OMNeT++ and publish most important data to
// other nodes
void OmnetReceiver::cam_callback(
    const etsi_its_cam_msgs::msg::CAM::SharedPtr msg) {
  etsi_its_cam_msgs::msg::CAM
      cacc_msg;  // Outgoing message is also etsi_its_cam_msgs::msg::CAM

  // --- Populate cacc_msg (ETSI CAM) from msg (ETSI CAM) ---

  // 1. Header (ItsPduHeader)
  // The incoming msg->header is ItsPduHeader. The outgoing cacc_msg.header is
  // also ItsPduHeader.
  cacc_msg.header.protocol_version =
      msg->header.protocol_version;  // Pass through or set default
  cacc_msg.header.message_id =
      msg->header.message_id;  // Pass through or set default
  cacc_msg.header.station_id.value =
      msg->header.station_id.value;  // Pass through station_id

  // 2. CAM Payload (etsi_its_cam_msgs::msg::CamPayload)
  cacc_msg.cam.generation_delta_time.value =
      msg->cam.generation_delta_time.value;  // Pass through

  // 3. Basic Container (etsi_its_cam_msgs::msg::BasicContainer)
  auto& cacc_bc = cacc_msg.cam.cam_parameters.basic_container;
  auto const& msg_bc = msg->cam.cam_parameters.basic_container;

  cacc_bc.station_type.value = msg_bc.station_type.value;  // Pass through

  // Apply original scaling to the *values* before assigning to the outgoing
  // ETSI CAM fields. The original comments (e.g., //0.1m) referred to the
  // *target unit/meaning* of the auna_its_msgs fields. Now we are populating
  // ETSI CAM fields, which have their own defined units. We assume the incoming
  // msg fields are raw values from OMNeT++ that need this scaling to match the
  // magnitudes that were previously stored in auna_its_msgs.

  // Longitude (ETSI unit: 1/10 microdegree)
  // Original logic: (float)value / 10 / scale_factor_
  cacc_bc.reference_position.longitude.value = static_cast<int32_t>(
      static_cast<float>(msg_bc.reference_position.longitude.value) / 10.0f /
      scale_factor_);
  // Latitude (ETSI unit: 1/10 microdegree)
  cacc_bc.reference_position.latitude.value = static_cast<int32_t>(
      static_cast<float>(msg_bc.reference_position.latitude.value) / 10.0f /
      scale_factor_);
  // Altitude (ETSI unit: 0.01m)
  // Original logic: (float)value / 100 / scale_factor_
  cacc_bc.reference_position.altitude.altitude_value.value =
      static_cast<int32_t>(
          static_cast<float>(
              msg_bc.reference_position.altitude.altitude_value.value) /
          100.0f / scale_factor_);

  cacc_bc.reference_position.altitude.altitude_confidence.value =
      msg_bc.reference_position.altitude.altitude_confidence
          .value;  // Pass through
  cacc_bc.reference_position.position_confidence_ellipse =
      msg_bc.reference_position
          .position_confidence_ellipse;  // Pass through struct

  // 4. High Frequency Container
  // (etsi_its_cam_msgs::msg::HighFrequencyContainer)
  cacc_msg.cam.cam_parameters.high_frequency_container.choice =
      msg->cam.cam_parameters.high_frequency_container
          .choice;  // Pass through choice

  if (msg->cam.cam_parameters.high_frequency_container.choice ==
      etsi_its_cam_msgs::msg::HighFrequencyContainer::
          CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY) {
    auto& cacc_vhf = cacc_msg.cam.cam_parameters.high_frequency_container
                         .basic_vehicle_container_high_frequency;
    auto const& msg_vhf = msg->cam.cam_parameters.high_frequency_container
                              .basic_vehicle_container_high_frequency;

    // Heading (ETSI unit: 0.1 degree)
    // Original logic for auna_its_msgs.theta (radians): (((float)heading_val /
    // 10) * M_PI / 180 - (2*M_PI)*floor(...)) Now, target is
    // cacc_vhf.heading.heading_value (0.1 degree). Assuming
    // msg_vhf.heading.heading_value.value is the raw value needing scaling. The
    // original scaling was effectively (value_in_0_1_deg / 10).
    cacc_vhf.heading.heading_value.value = static_cast<uint16_t>(
        static_cast<float>(msg_vhf.heading.heading_value.value) /
        10.0f /* scale_factor_ not applied to heading in transmitter */);
    cacc_vhf.heading.heading_confidence.value =
        msg_vhf.heading.heading_confidence.value;  // Pass through

    // Yaw Rate (ETSI unit: 0.01 degree/s)
    // Original logic for auna_its_msgs.thetadot (rad/s): ((float)yaw_rate_val /
    // 100) * M_PI / 180 Target is cacc_vhf.yaw_rate.yaw_rate_value (0.01
    // degree/s). Original scaling was (value_in_0_01_deg_per_s / 100).
    cacc_vhf.yaw_rate.yaw_rate_value.value = static_cast<int16_t>(
        static_cast<float>(msg_vhf.yaw_rate.yaw_rate_value.value) /
        100.0f /* scale_factor_ not applied here in transmitter */);
    cacc_vhf.yaw_rate.yaw_rate_confidence.value =
        msg_vhf.yaw_rate.yaw_rate_confidence.value;  // Pass through

    // Speed (ETSI unit: 0.01 m/s)
    // Original logic: (float)value / 100 / scale_factor_
    cacc_vhf.speed.speed_value.value = static_cast<uint16_t>(
        static_cast<float>(msg_vhf.speed.speed_value.value) / 100.0f /
        scale_factor_);
    cacc_vhf.speed.speed_confidence.value =
        msg_vhf.speed.speed_confidence.value;  // Pass through

    // Longitudinal Acceleration (ETSI unit: 0.1 m/s^2)
    // Original logic: (float)value / 10 / scale_factor_
    cacc_vhf.longitudinal_acceleration.longitudinal_acceleration_value.value =
        static_cast<int16_t>(
            static_cast<float>(msg_vhf.longitudinal_acceleration
                                   .longitudinal_acceleration_value.value) /
            10.0f / scale_factor_);
    cacc_vhf.longitudinal_acceleration.longitudinal_acceleration_confidence
        .value =
        msg_vhf.longitudinal_acceleration.longitudinal_acceleration_confidence
            .value;  // Pass through

    // Pass through other VHF fields directly as their structure is the same
    // and scaling was not applied to these in the original auna_its_msgs
    // mapping for receiver.
    cacc_vhf.drive_direction.value = msg_vhf.drive_direction.value;
    cacc_vhf.vehicle_length = msg_vhf.vehicle_length;  // Struct copy
    cacc_vhf.vehicle_width.value =
        msg_vhf.vehicle_width.value;         // Assuming VehicleWidth is simple
    cacc_vhf.curvature = msg_vhf.curvature;  // Struct copy
    cacc_vhf.curvature_calculation_mode.value =
        msg_vhf.curvature_calculation_mode.value;
    // Note: vehicle_length, vehicle_width, curvature were not explicitly set in
    // original OmnetReceiver's auna_its_msgs. They are present in
    // etsi_its_cam_msgs, so passing them through.
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Received CAM without BasicVehicleContainerHighFrequency. "
                "Populating limited CAM.");
  }

  cam_publisher_->publish(cacc_msg);
  cam_publisher_->publish(cacc_msg);
}
