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

#include "auna_template/sample_node.hpp"

// GazeboModels::GazeboModels(): Node("gazebo_models_node")
// {
//     subscription_ =
//     this->create_subscription<gazebo_msgs::msg::ModelStates>("/model_states",
//     2, [this](const gazebo_msgs::msg::ModelStates::SharedPtr
//     msg){model_state_callback(msg);}); publisher_ =
//     this->create_publisher<auna_msgs::msg::StringArray>("/robot_names", 2);
// }

// void GazeboModels::model_state_callback(const
// gazebo_msgs::msg::ModelStates::SharedPtr msg){
//     auto message = auna_msgs::msg::StringArray();
//     for(std::string model_name : msg->name)
//     {
//         if(model_name.find("robot") != std::string::npos)
//         {
//             message.strings.push_back(model_name);
//         }
//     }
//     publisher_->publish(message);
// }
