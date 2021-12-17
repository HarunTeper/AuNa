#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "gazebo_msgs/srv/get_model_list.hpp"

using namespace std::placeholders;


// Creates a node that subscribes to all robot tf topics and publishes them to the global tf topic.
class GlobalTF : public rclcpp::Node
{
    public:
        // Create a service callback to the Gazebo models to get the current robot names.
        GlobalTF()
        : Node("global_tf_node"), tf_broadcaster(this)
        {
            std::chrono::duration<double> timer_duration = std::chrono::duration<double>(1.0);
            service_timer = this->create_wall_timer(timer_duration,std::bind(&GlobalTF::service_timer_callback,this));
            modelClient = this->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
        }

    private:

        // Callback for the timer. Creates a service call to get the Gazebo model names.
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
            auto result = modelClient->async_send_request(request,std::bind(&GlobalTF::model_srv_callback,this,std::placeholders::_1));
        }

        // Service callback of the Gazebo models.
        void model_srv_callback(const rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedFuture future)
        {
            auto result = future.get();
            for(std::string model_name : result.get()->model_names)
            {
                // Only check for names that include robot
                if(model_name.find("robot") != std::string::npos)
                {
                    // For each model name, check if already added. If not, add subscribers for local tf topics.
                    if (std::find(robot_models.begin(), robot_models.end(), model_name) == robot_models.end()) {
                        robot_models.push_back(model_name);
                        tf_subscribers.push_back(this->create_subscription<tf2_msgs::msg::TFMessage>(model_name+"/tf", 10, std::bind(&GlobalTF::tf_callback, this, _1)));
                        tf_subscribers.push_back(this->create_subscription<tf2_msgs::msg::TFMessage>(model_name+"/tf_static", 10, std::bind(&GlobalTF::tf_callback, this, _1)));
                    }
                }
            }
        }

        // Callback for local tf topics. Publishes the transformations to the global tf topic.
        void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
        {
            for(const geometry_msgs::msg::TransformStamped message : msg->transforms)
            {
                tf_broadcaster.sendTransform(message);
            }
        }

        // TF Broadcaster
        tf2_ros::TransformBroadcaster tf_broadcaster;

        // Timer and Service Client
        rclcpp::TimerBase::SharedPtr service_timer;
        rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr modelClient;

        // Robot model vector and subscribers for local tf topics.
        std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> tf_subscribers;
        std::vector<std::string> robot_models;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalTF>());
    rclcpp::shutdown();
    return 0;
}
