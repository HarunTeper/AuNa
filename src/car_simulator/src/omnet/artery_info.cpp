#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

using namespace std::placeholders;

// Creates a node that requests the currently simulated robots in Gazebo and publishes their namespaces to Artery and OMNeT++
class arteryInfo : public rclcpp::Node
{
    public:
        // Create the publisher, timer and service client
        arteryInfo(): Node("artery_info_node"), robot_names()
        {
            subscription = this->create_subscription<gazebo_msgs::msg::ModelStates>("/model_states", 10, std::bind(&arteryInfo::model_state_callback, this, _1));
            publisher = this->create_publisher<std_msgs::msg::String>("/artery_model_list", 10);
        }

    private:
        // Timer callback which starts the service request to Gazebo

        void model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg){
            for(std::string model_name : msg->name)
            {
                if(model_name.find("robot") != std::string::npos)
                {
                    auto message = std_msgs::msg::String();
                    message.data = model_name;
                    publisher->publish(message);
                }
            }
        }

        rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr service_timer;
        std::vector<std::string> robot_names;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<arteryInfo>());

    rclcpp::shutdown();
    return 0;
}