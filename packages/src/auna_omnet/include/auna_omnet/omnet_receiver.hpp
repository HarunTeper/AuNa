
#include "rclcpp/rclcpp.hpp"

#include "auna_its_msgs/msg/cam.hpp"

class OmnetReceiver : public rclcpp::Node
{
public:
  OmnetReceiver();

private:
  void cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg);

  rclcpp::Publisher<auna_its_msgs::msg::CAM>::SharedPtr cam_publisher_;
  rclcpp::Subscription<auna_its_msgs::msg::CAM>::SharedPtr omnet_subscriber_;
  double scale_factor_ = 10;
};