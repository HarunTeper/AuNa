// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <mqtt/async_client.h>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher;
class MqttCallback : public virtual mqtt::callback
{
      MinimalPublisher * m_obj;

public:
    MqttCallback(MinimalPublisher * _obj);
    ~MqttCallback(){}

    void message_arrived(mqtt::const_message_ptr msg);
};

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("mqtt_repost", 10);

    const std::string MQTT_BROKER_ADDRESS = "localhost:1883";
    const std::string MQTT_CLIENT_ID = "LIMoSim";
    const std::string _topic = "mqtt_in";

    m_mqttClient_ = new mqtt::async_client(MQTT_BROKER_ADDRESS, MQTT_CLIENT_ID);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);
    MqttCallback * m_callback = new MqttCallback(this);
    m_mqttClient_->set_callback(*m_callback);

    try {
        m_mqttClient_->connect(connOpts)->wait();
        //  m_mqttClient_.disconnect()->wait();
    }
    catch (const mqtt::exception& ex) {
        std::cout << ex << std::endl;
    }
    m_mqttClient_->subscribe(_topic, 0)->wait();

  }
void mqtt_callback(nlohmann::json data){
    auto message = std_msgs::msg::String();
    message.data = data.dump();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
private:

  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  mqtt::async_client * m_mqttClient_;
  MqttCallback * m_callback;  
  // size_t count_;
};
MqttCallback::MqttCallback(MinimalPublisher * _obj){
        m_obj = _obj;
    }

void MqttCallback::message_arrived(mqtt::const_message_ptr msg)
    {
      nlohmann::json data = nlohmann::json::parse(msg->get_payload_str());
      m_obj->mqtt_callback(data);
    }


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
