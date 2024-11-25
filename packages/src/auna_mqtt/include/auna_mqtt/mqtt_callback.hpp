#ifndef AUNA_MQTT_CALLBACK_HPP
#define AUNA_MQTT_CALLBACK_HPP

#include "rclcpp/rclcpp.hpp"
#include "mqtt/async_client.h"
#include "nlohmann/json.hpp"
// #include "auna_mqtt/mqtt_waypoint_receiver.hpp"

class MQTTWaypointReceiver;
class MqttCallback : public virtual mqtt::callback
{
    MQTTWaypointReceiver * m_obj;

    public:
        MqttCallback(MQTTWaypointReceiver * _obj);
        ~MqttCallback(){}

        void message_arrived(mqtt::const_message_ptr msg);
};

#endif