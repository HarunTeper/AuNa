#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "ros_its_msgs/msg/cam.hpp"

#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"

using namespace std::placeholders;

// Creates a node which requests the current pose of the robot in the simulation and publishes it to simulation_pose
class SimulationPose : public rclcpp::Node
{
    public:
        // Create the publisher, timer and service client
        SimulationPose(std::string name): Node("model_state_node")
        {
            publisher = this->create_publisher<ros_its_msgs::msg::CAM>("simulation_pose", 10);
            modelClient = this->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
            std::chrono::duration<double> timer_duration = std::chrono::duration<double>(publish_rate);
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

            ros_its_msgs::msg::CAM simulation_pose;
            simulation_pose.header.frame_id = this->name;
            simulation_pose.header.stamp = entity->header.stamp;

            // Adjust for robot size
            simulation_pose.x = entity->state.pose.position.x*10*scale_factor;//0.1m
            simulation_pose.y = entity->state.pose.position.y*10*scale_factor;//0.1m
            simulation_pose.z = entity->state.pose.position.z*100*scale_factor;//0.01m

            std::vector<double> rpy = euler_from_quaternion(std::vector<double>{entity->state.pose.orientation.x,entity->state.pose.orientation.y,entity->state.pose.orientation.z,entity->state.pose.orientation.w});
            simulation_pose.theta = rpy[2] * 180 / M_PI - 360 * floor( rpy[2] * 180 / M_PI / 360 ); // get heading in radians //1 degree
            simulation_pose.thetadot = entity->state.twist.angular.z * 180 / M_PI;//1 yydegree/s

            // Determine the speed and drive direction, in reference to the global frame
            float old_speed = this->speed;

            float len_x = entity->state.twist.linear.x/(abs(entity->state.twist.linear.x)+abs(entity->state.twist.linear.y));
            float len_y = entity->state.twist.linear.y/(abs(entity->state.twist.linear.x)+abs(entity->state.twist.linear.y));
            float len_h = sqrt(pow(len_x,2)+pow(len_y,2));
            float velocity_heading = acos(len_x/len_h) * 180 / M_PI;
            velocity_heading = (len_y>=0) * velocity_heading + (len_y<0)* (360-velocity_heading);
            int drive_direction = simulation_pose.theta-velocity_heading>90 || simulation_pose.theta-velocity_heading<270;
            this->speed = sqrt(pow(entity->state.twist.linear.x,2)+pow(entity->state.twist.linear.y,2)) * scale_factor; //1 m/s

            simulation_pose.drive_direction = drive_direction;
            simulation_pose.v = this->speed;
            simulation_pose.vdot = (this->speed-old_speed)/publish_rate*scale_factor;//1 m/s^2

            simulation_pose.curv = simulation_pose.thetadot/std::max(0.01,simulation_pose.v)/scale_factor;//1/m

            simulation_pose.vehicle_length = 0.49*scale_factor;
            simulation_pose.vehicle_width = 0.18*scale_factor;

            publisher->publish(simulation_pose);
        }

        // Converts to Euler angles from quaternion representation of orientation
        std::vector<double> euler_from_quaternion(std::vector<double> quaternion)
        {
            double x = quaternion[0];
            double y = quaternion[1];
            double z = quaternion[2];
            double w = quaternion[3];

            double roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            double pitch = asin(2 * (w * y - z * x));
            double yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

            return std::vector<double>{roll,pitch,yaw};
        }

        rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr modelClient;
        rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription;
        rclcpp::Publisher<ros_its_msgs::msg::CAM>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr service_timer;
        std::string name;
        double speed;
        double publish_rate = 0.1;
        double scale_factor = 10;
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