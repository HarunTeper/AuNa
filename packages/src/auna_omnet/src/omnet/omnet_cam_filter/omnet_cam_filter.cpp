#include "auna_omnet/omnet_cam_filter.hpp"

OmnetCamFilter::OmnetCamFilter(int identifier) : Node("omnet_cam_filter_node")
{
    cam_subscriber_ = this->create_subscription<auna_its_msgs::msg::CAM>("cam", 2, [this](const auna_its_msgs::msg::CAM::SharedPtr msg){cam_callback(msg);});
    cam_publisher_ = this->create_publisher<auna_its_msgs::msg::CAM>("cam_filtered", 2);
    service_ = this->create_service<auna_msgs::srv::Identifier>("cam_filter_index", [this](const std::shared_ptr<auna_msgs::srv::Identifier::Request> request, std::shared_ptr<auna_msgs::srv::Identifier::Response> response){filter_service_callback(request,response);});
    identifier_ = identifier;
}

// Set the identifier for filtering CAMs
void OmnetCamFilter::filter_service_callback(const std::shared_ptr<auna_msgs::srv::Identifier::Request> request, std::shared_ptr<auna_msgs::srv::Identifier::Response> response){
    identifier_=request->identifier;
    response->success=true;
}

// Filter CAMs using the identifier
void OmnetCamFilter::cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg)
{
    std::string filtered_string;
    std::copy_if(msg->robot_name.begin(), msg->robot_name.end(),std::back_inserter(filtered_string),[this](char ch){return '0' <= ch && ch <= '9';});
    if(std::stoi(filtered_string) == identifier_){
        cam_publisher_->publish(*msg);
    }
}
