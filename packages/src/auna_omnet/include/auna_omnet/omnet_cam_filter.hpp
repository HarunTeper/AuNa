
#include "rclcpp/rclcpp.hpp"

#include "auna_msgs/srv/identifier.hpp"
#include "etsi_its_cam_msgs/msg/cam.hpp"

class OmnetCamFilter : public rclcpp::Node
{
public:
  OmnetCamFilter(int identifier);

private:
  void cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg);
  void filter_service_callback(
    const std::shared_ptr<auna_msgs::srv::Identifier::Request> request,
    std::shared_ptr<auna_msgs::srv::Identifier::Response> response);
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_publisher_;
  rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_subscriber_;
  rclcpp::Service<auna_msgs::srv::Identifier>::SharedPtr service_;

  int identifier_ = 0;
};