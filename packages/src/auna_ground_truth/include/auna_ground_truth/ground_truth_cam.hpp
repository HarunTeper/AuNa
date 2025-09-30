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

#ifndef AUNA_GROUND_TRUTH__GROUND_TRUTH_CAM_HPP_
#define AUNA_GROUND_TRUTH__GROUND_TRUTH_CAM_HPP_

#include "rclcpp/rclcpp.hpp"

#include <etsi_its_msgs_utils/cam_access.hpp>  // access functions

#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "std_msgs/msg/header.hpp"
#include <etsi_its_cam_msgs/msg/cam.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>

class GroundTruthCam : public rclcpp::Node
{
public:
  GroundTruthCam();

private:
  void service_timer_callback();
  void model_srv_callback(
    const rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future);

  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr modelClient_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr service_timer_;

  std::string name_;
  double speed_;
  int publish_milliseconds_ = 100;
  double scale_factor_ = 10;
};

#endif  // AUNA_GROUND_TRUTH__GROUND_TRUTH_CAM_HPP_

