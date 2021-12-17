#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"

using namespace std::placeholders;

// Creates a node that requests the currently simulated robots in Gazebo and publishes their namespaces to Artery and OMNeT++
class arteryInfo : public rclcpp::Node
{
    public:
        // Create the publisher, timer and service client
        arteryInfo(): Node("artery_info_node"), robot_names()
        {
            publisher = this->create_publisher<std_msgs::msg::String>("/model_states", 10);
            modelClient = this->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
            std::chrono::duration<double> timer_duration = std::chrono::duration<double>(1.0 / 100.0);
            service_timer = this->create_wall_timer(timer_duration, std::bind(&arteryInfo::service_timer_callback, this));
        }

    private:
        // Timer callback which starts the service request to Gazebo
        void service_timer_callback()
        {
            auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
            while (!modelClient->wait_for_service()) 
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto result = modelClient->async_send_request(request,std::bind(&arteryInfo::model_srv_callback,this,std::placeholders::_1));
        }

        // Service callback which publishes the model namespaces to Artery and OMNeT++
        void model_srv_callback(const rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedFuture future)
        {
            auto result = future.get();
            for(std::string model_name : result.get()->model_names)
            {
                if(model_name.find("robot") != std::string::npos)
                {
                    auto message = std_msgs::msg::String();
                    message.data = model_name;
                    publisher->publish(message);
                }
            }
        }

        rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr modelClient;
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