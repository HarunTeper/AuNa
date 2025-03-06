#ifndef CAM_RECEIVER_HPP_
#define CAM_RECEIVER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <etsi_its_cam_msgs/msg/cam.hpp>

class CamReceiver : public rclcpp::Node
{
public:
  CamReceiver();

private:
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr publisher_;
  rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr subscription_;
};

#endif  // CAM_RECEIVER_HPP_