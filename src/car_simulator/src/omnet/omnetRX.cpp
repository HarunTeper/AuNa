#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "etsi_its_msgs/msg/cam.hpp"
#include "ros_its_msgs/msg/cam.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

// Robot receiver node to receive CAMs from Artery and OMNeT++
class omnetRX : public rclcpp::Node
{
    public:
        // Create a publisher and subscriber
        omnetRX() : Node("omnetRX")
        {
            cam_publisher = this->create_publisher<ros_its_msgs::msg::CAM>("caccCam", 10);
            omnet_subscriber = this->create_subscription<etsi_its_msgs::msg::CAM>("camRX", 10, std::bind(&omnetRX::cam_callback, this, _1));
        }

    private:

        // Receive the CAM from Artery and OMNeT++ and publish most important data to other nodes
        void cam_callback(const etsi_its_msgs::msg::CAM::SharedPtr msg)
        {
            ros_its_msgs::msg::CAM cacc_msg;

            cacc_msg.x = (float)msg->reference_position.longitude/10/scale_factor; //0.1m
            cacc_msg.y = (float)msg->reference_position.latitude/10/scale_factor; //0.1m
            cacc_msg.theta = (((float)msg->high_frequency_container.heading.value/10)*M_PI/180 - (2*M_PI) * floor(((float)msg->high_frequency_container.heading.value/10)*M_PI/180 / (M_PI)));//0.1 degree
            cacc_msg.thetadot = (msg->high_frequency_container.yaw_rate.value/100)*M_PI/180;//0.01degree/s
            cacc_msg.v = (float)msg->high_frequency_container.speed.value/100/scale_factor;//0.01 m/s
            cacc_msg.vdot = (float)msg->high_frequency_container.longitudinal_acceleration.value/10/scale_factor;//0.1m/s^2

            cam_publisher->publish(cacc_msg);
        }

        rclcpp::Publisher<ros_its_msgs::msg::CAM>::SharedPtr cam_publisher;
        rclcpp::Subscription<etsi_its_msgs::msg::CAM>::SharedPtr omnet_subscriber;
        double scale_factor = 10;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<omnetRX>());

    rclcpp::shutdown();
    return 0;
}