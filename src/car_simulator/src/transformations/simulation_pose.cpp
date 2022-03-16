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

#include "gazebo_msgs/srv/get_model_list.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::placeholders;

// Creates a node which requests the current pose of the robot in the simulation and publishes it to simulation_pose
class SimulationPose : public rclcpp::Node
{
    public:
        // Create the publisher, timer and service client
        SimulationPose(std::string name): Node("model_state_node")
        {
            publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("simulation_pose", 10);
            modelClient = this->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
            std::chrono::duration<double> timer_duration = std::chrono::duration<double>(1.0 / 100.0);
            service_timer = this->create_wall_timer(timer_duration, std::bind(&SimulationPose::service_timer_callback, this));
            this->name = name;
        }

    private:
        // Timer callback to periodically call a service request for the model state
        void service_timer_callback()
        {
            auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
            request->name = name;
            while (!modelClient->wait_for_service()) 
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto result = modelClient->async_send_request(request,std::bind(&SimulationPose::model_srv_callback,this,std::placeholders::_1));
        }

        // Read the requested entity state and publish the received pose to simulation_pose
        void model_srv_callback(const rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future)
        {
            auto result = future.get();
            auto entity = result.get();
            geometry_msgs::msg::PoseStamped simulation_pose;
            simulation_pose.header.frame_id = name;
            simulation_pose.header.stamp = entity->header.stamp;
            simulation_pose.pose = entity->state.pose;
            publisher->publish(simulation_pose);
        }

        rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr modelClient;
        rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr service_timer;
        std::string name;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    

    // Set name for request, if first argument is non-empty.
    if(argc > 1)
    {
      rclcpp::spin(std::make_shared<SimulationPose>(argv[1]));
    }
    else
    {
      rclcpp::spin(std::make_shared<SimulationPose>(""));
    }

    rclcpp::shutdown();
    return 0;
}