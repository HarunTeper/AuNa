#include "rclcpp/rclcpp.hpp"

#include "ros_its_msgs/msg/cam.hpp"
#include "ros_its_msgs/srv/identifier.hpp"

class OmnetCamFilter : public rclcpp::Node
{
    public:
        OmnetCamFilter(int identifier) : Node("omnet_cam_filter_node")
        {
            cam_subscriber_ = this->create_subscription<ros_its_msgs::msg::CAM>("caccCam", 2, [this](const ros_its_msgs::msg::CAM::SharedPtr msg){cam_callback(msg);});
            cam_publisher_ = this->create_publisher<ros_its_msgs::msg::CAM>("cam_filtered", 2);
            service_ = this->create_service<ros_its_msgs::srv::Identifier>("cam_filter_index", [this](const std::shared_ptr<ros_its_msgs::srv::Identifier::Request> request, std::shared_ptr<ros_its_msgs::srv::Identifier::Response> response){filter_service_callback(request,response);});
            identifier_ = identifier;
        }
    private:
        void cam_callback(const ros_its_msgs::msg::CAM::SharedPtr msg)
        {
            if(std::stoi(msg->robot_name) == identifier_){
                cam_publisher_->publish(*msg);
            }
        }
        void filter_service_callback(const std::shared_ptr<ros_its_msgs::srv::Identifier::Request> request, std::shared_ptr<ros_its_msgs::srv::Identifier::Response> response)
        {
            identifier_=request->identifier;
            response->success=true;
        }
        rclcpp::Publisher<ros_its_msgs::msg::CAM>::SharedPtr cam_publisher_;
        rclcpp::Subscription<ros_its_msgs::msg::CAM>::SharedPtr cam_subscriber_;
        rclcpp::Service<ros_its_msgs::srv::Identifier>::SharedPtr service_;

        int identifier_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Set prefix for topic and frame_id, if first argument is non-empty.
    if(argc > 1)
    {
      rclcpp::spin(std::make_shared<OmnetCamFilter>(std::stoi(argv[1])));
    }
    else
    {
      rclcpp::spin(std::make_shared<OmnetCamFilter>(0));
    }
    rclcpp::shutdown();
    return 0;
}
