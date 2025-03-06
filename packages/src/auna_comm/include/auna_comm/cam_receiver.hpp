#ifndef AUNA_COMM_CAM_RECEIVER_HPP
#define AUNA_COMM_CAM_RECEIVER_HPP

#include "rclcpp/rclcpp.hpp"

#include "etsi_its_cam_msgs/msg/cam.hpp"

#include <memory>

class CamReceiver : public rclcpp::Node
{
public:
  CamReceiver();

private:
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr publisher_;
  rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr subscription_;
};

#endif  // AUNA_COMM_CAM_RECEIVER_HPP