// Copyright 2022 The MathWorks, Inc.

#ifndef _SLROS2_GENERIC_PUBSUB_H_
#define _SLROS2_GENERIC_PUBSUB_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "slros_busmsg_conversion.h"

extern rclcpp::Node::SharedPtr SLROSNodePtr;
inline rclcpp::QoS getQOSSettingsFromRMW(const rmw_qos_profile_t& qosProfile);

/**
 * Class for subscribing to ROS 2 messages in C++.
 *
 * This class is used by code generated from the Simulink ROS 2
 * subscriber blocks and is templatized by the ROS 2 message type and
 * Simulink bus type.
 */
template <class MsgType, class BusType>
class SimulinkSubscriber {
  public:
    void createSubscriber(std::string const& topic, const rmw_qos_profile_t& qosProfile);
    bool getLatestMessage(BusType* busPtr); // returns true iff message is new
  private:
    std::shared_ptr<rclcpp::Subscription<MsgType>> _subscriber;
    bool _newMessageReceived;
    std::shared_ptr<MsgType> _lastMsg;
    std::mutex _subMutex;
};

/**
 * Class for publishing ROS 2 messages in C++.
 *
 * This class is used by code generated from the Simulink ROS 2
 * publisher blocks and is templatized by the ROS 2 message type and
 * Simulink bus type.
 */
template <class MsgType, class BusType>
class SimulinkPublisher {
  public:
    void createPublisher(std::string const& topic, const rmw_qos_profile_t& qosProfile);
    void publish(const BusType* busPtr); // returns true iff message is new
  private:
    std::shared_ptr<rclcpp::Publisher<MsgType>> _publisher;
    std::shared_ptr<MsgType> _lastMsg;
};


/**
 * Create a C++ subscriber object
 *
 * @param topic The topic name to subscribe to
 * @param qosProfile QoS profile of the subscription
 */
template <class MsgType, class BusType>
void SimulinkSubscriber<MsgType, BusType>::createSubscriber(std::string const& topic,
                                                            const rmw_qos_profile_t& qosProfile) {
    auto callback = [this](std::shared_ptr<MsgType> msg) {
        std::lock_guard<std::mutex> lockMsg(_subMutex);
        _lastMsg = msg;
    };
    _subscriber = SLROSNodePtr->create_subscription<MsgType>(
        topic, getQOSSettingsFromRMW(qosProfile), callback);
}

/**
 * Get the latest received message
 *
 * @param busPtr Simulink bus structure that should be populated with message contents
 * @return =TRUE, then a new message has been received and *busPtr holds the newly-received message.
 * =FALSE when a new message has not been received and *busPtr is unchanged.
 */
template <class MsgType, class BusType>
bool SimulinkSubscriber<MsgType, BusType>::getLatestMessage(BusType* busPtr) {
    if (_lastMsg.get()) {
        std::lock_guard<std::mutex> lockMsg(_subMutex);
        convertToBus(busPtr, *_lastMsg);
        _lastMsg.reset();
        return true;
    }
    return false;
}

/**
 * Create a C++ publisher object
 *
 * @param topic The topic name to publish to
 * @param qosProfile QoS profile of the publisher
 */
template <class MsgType, class BusType>
void SimulinkPublisher<MsgType, BusType>::createPublisher(std::string const& topic,
                                                          const rmw_qos_profile_t& qosProfile) {
    _publisher = SLROSNodePtr->create_publisher<MsgType>(topic, getQOSSettingsFromRMW(qosProfile));
}

/**
 * Publish a message
 *
 * @param busPtr Pointer to the bus structure for the outgoing message
 */
template <class MsgType, class BusType>
void SimulinkPublisher<MsgType, BusType>::publish(const BusType* inBus) {
    auto msg = std::make_unique<MsgType>();
    convertFromBus(*msg, inBus);
    _publisher->publish(std::move(msg));
}

#endif
