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

#ifndef AUNA_GAZEBO__ROBOT_NAME_PUBLISHER_HPP_
#define AUNA_GAZEBO__ROBOT_NAME_PUBLISHER_HPP_

#include "auna_msgs/msg/string_array.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotNamePublisher : public rclcpp::Node
{
public:
  RobotNamePublisher();

private:
  void model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
  rclcpp::Publisher<auna_msgs::msg::StringArray>::SharedPtr publisher_;
};

#endif  // AUNA_GAZEBO__ROBOT_NAME_PUBLISHER_HPP_
