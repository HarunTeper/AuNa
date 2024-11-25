
#include "rclcpp/rclcpp.hpp"

#include "auna_its_msgs/msg/cam.hpp"
#include "auna_msgs/srv/identifier.hpp"

class OmnetCamFilter : public rclcpp::Node
{
    public:
        OmnetCamFilter(int identifier);
    private:
        void cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg);
        void filter_service_callback(const std::shared_ptr<auna_msgs::srv::Identifier::Request> request, std::shared_ptr<auna_msgs::srv::Identifier::Response> response);
        rclcpp::Publisher<auna_its_msgs::msg::CAM>::SharedPtr cam_publisher_;
        rclcpp::Subscription<auna_its_msgs::msg::CAM>::SharedPtr cam_subscriber_;
        rclcpp::Service<auna_msgs::srv::Identifier>::SharedPtr service_;

        int identifier_ = 0;
};