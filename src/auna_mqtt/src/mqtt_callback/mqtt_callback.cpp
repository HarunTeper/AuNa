#include "auna_mqtt/mqtt_callback.hpp"
#include "auna_mqtt/mqtt_waypoint_receiver.hpp"


MqttCallback::MqttCallback(MQTTWaypointReceiver * _obj)
{
    m_obj = _obj;
}

void MqttCallback::message_arrived(mqtt::const_message_ptr msg)
{
    nlohmann::json data = nlohmann::json::parse(msg->get_payload_str());
    m_obj->mqtt_callback(data);
}
