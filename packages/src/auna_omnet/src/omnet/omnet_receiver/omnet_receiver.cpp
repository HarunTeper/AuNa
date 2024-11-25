#include "auna_omnet/omnet_receiver.hpp"

// Create a publisher and subscriber
OmnetReceiver::OmnetReceiver() : Node("omnet_receiver_node")
{
    omnet_subscriber_ = this->create_subscription<etsi_its_msgs::msg::CAM>("cam_in", 2, [this](const etsi_its_msgs::msg::CAM::SharedPtr msg){cam_callback(msg);});
    cam_publisher_ = this->create_publisher<auna_its_msgs::msg::CAM>("cam", 2);
}

// Receive the CAM from Artery and OMNeT++ and publish most important data to other nodes
void OmnetReceiver::cam_callback(const etsi_its_msgs::msg::CAM::SharedPtr msg)
{
    auna_its_msgs::msg::CAM cacc_msg;

    cacc_msg.robot_name = msg->header.frame_id;
    cacc_msg.header.stamp = msg->header.stamp;

    cacc_msg.x = (float)msg->reference_position.longitude/10/scale_factor_; //0.1m
    cacc_msg.y = (float)msg->reference_position.latitude/10/scale_factor_; //0.1m
    cacc_msg.theta = (((float)msg->high_frequency_container.heading.value/10)*M_PI/180 - (2*M_PI) * floor(((float)msg->high_frequency_container.heading.value/10)*M_PI/180 / (M_PI)));//0.1 degree
    cacc_msg.thetadot = ((float)msg->high_frequency_container.yaw_rate.value/100)*M_PI/180;//0.01degree/s
    cacc_msg.v = (float)msg->high_frequency_container.speed.value/100/scale_factor_;//0.01 m/s
    cacc_msg.vdot = (float)msg->high_frequency_container.longitudinal_acceleration.value/10/scale_factor_;//0.1m/s^2

    cam_publisher_->publish(cacc_msg);
}
