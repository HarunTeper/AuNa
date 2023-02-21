// Copyright 2022 The MathWorks, Inc.
// Generated 21-Feb-2023 12:19:03
#ifndef _SLROS2_INITIALIZE_H_
#define _SLROS2_INITIALIZE_H_
#include "CACC_Center_types.h"
// Generic pub-sub header
#include "slros2_generic_pubsub.h"
// Generic service header
#include "slros2_generic_service.h"
extern rclcpp::Node::SharedPtr SLROSNodePtr;
#ifndef SET_QOS_VALUES
#define SET_QOS_VALUES(qosStruct, hist, dep, dur, rel)  \
    {                                                   \
        qosStruct.history = hist;                       \
        qosStruct.depth = dep;                          \
        qosStruct.durability = dur;                     \
        qosStruct.reliability = rel;                    \
    }
#endif
inline rclcpp::QoS getQOSSettingsFromRMW(const rmw_qos_profile_t& qosProfile) {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(qosProfile));
    if (RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL == qosProfile.durability) {
        qos.transient_local();
    } else {
        qos.durability_volatile();
    }
    if (RMW_QOS_POLICY_RELIABILITY_RELIABLE == qosProfile.reliability) {
        qos.reliable();
    } else {
        qos.best_effort();
    }
    return qos;
}
// CACC_Center/publisher/Publish
extern SimulinkPublisher<geometry_msgs::msg::Twist,SL_Bus_geometry_msgs_Twist> Pub_CACC_Center_2215;
// CACC_Center/controller/LA/Subscribe
extern SimulinkSubscriber<example_interfaces::msg::Float64,SL_Bus_example_interfaces_Float64> Sub_CACC_Center_2234;
// CACC_Center/controller/LA/Subscribe1
extern SimulinkSubscriber<example_interfaces::msg::Float64,SL_Bus_example_interfaces_Float64> Sub_CACC_Center_2273;
// CACC_Center/publisher/Subscribe
extern SimulinkSubscriber<geometry_msgs::msg::PoseStamped,SL_Bus_geometry_msgs_PoseStamped> Sub_CACC_Center_2221;
// CACC_Center/publisher/Subscribe1
extern SimulinkSubscriber<nav_msgs::msg::Odometry,SL_Bus_nav_msgs_Odometry> Sub_CACC_Center_2222;
// CACC_Center/subscriber/Subscribe
extern SimulinkSubscriber<auna_its_msgs::msg::CAM,SL_Bus_auna_its_msgs_CAM> Sub_CACC_Center_2128;
#endif
