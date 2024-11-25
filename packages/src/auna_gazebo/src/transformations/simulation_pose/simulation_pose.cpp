#include "auna_gazebo/simulation_pose.hpp"

// Create the publisher, timer and service client
SimulationPose::SimulationPose(std::string name): Node("model_state_node")
{
    publisher_ = this->create_publisher<auna_its_msgs::msg::CAM>("simulation_pose", 2);
    modelClient_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
    service_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_milliseconds_), [this](){service_timer_callback();});
    this->name_ = name;
}

// Timer callback to periodically call a service request for the model state
void SimulationPose::service_timer_callback()
{
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = name_;
    while (!modelClient_->wait_for_service(std::chrono::seconds(5))) 
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto result = modelClient_->async_send_request(request,std::bind(&SimulationPose::model_srv_callback,this,std::placeholders::_1));
}

// Read the requested entity state and publish the received pose to simulation_pose
void SimulationPose::model_srv_callback(const rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future)
{
    auto result = future.get();
    auto entity = result.get();

    auna_its_msgs::msg::CAM simulation_pose;
    simulation_pose.header.frame_id = this->name_;
    simulation_pose.header.stamp = entity->header.stamp;

    // Adjust for robot size
    simulation_pose.x = entity->state.pose.position.x*scale_factor_;//0.1m
    simulation_pose.y = entity->state.pose.position.y*scale_factor_;//0.1m
    simulation_pose.z = entity->state.pose.position.z*scale_factor_;//0.01m

    // Determine yaw from orientation quaternion
    tf2::Quaternion quat(
        entity->state.pose.orientation.x,
        entity->state.pose.orientation.y,
        entity->state.pose.orientation.z,
        entity->state.pose.orientation.w
    );
    quat.normalize();
    tf2::Matrix3x3 matrix(quat);
    tf2Scalar roll;
    tf2Scalar pitch;
    tf2Scalar yaw;
    matrix.getRPY(roll, pitch, yaw);

    // Calculate theta and thetadot from yaw and angular velocity in degrees
    simulation_pose.theta = yaw * 180 / M_PI - 360 * floor( yaw * 180 / M_PI / 360 ); // get heading in radians //1 degree
    simulation_pose.thetadot = entity->state.twist.angular.z * 180 / M_PI;//1 degree/s

    // Determine the speed and drive direction, in reference to the global frame
    float old_speed = this->speed_;

    float len_x = entity->state.twist.linear.x/(abs(entity->state.twist.linear.x)+abs(entity->state.twist.linear.y));
    float len_y = entity->state.twist.linear.y/(abs(entity->state.twist.linear.x)+abs(entity->state.twist.linear.y));
    float len_h = sqrt(pow(len_x,2)+pow(len_y,2));
    float velocity_heading = acos(len_x/len_h) * 180 / M_PI;
    velocity_heading = (len_y>=0) * velocity_heading + (len_y<0)* (360-velocity_heading);
    int drive_direction = simulation_pose.theta-velocity_heading>90 || simulation_pose.theta-velocity_heading<270;
    this->speed_ = sqrt(pow(entity->state.twist.linear.x,2)+pow(entity->state.twist.linear.y,2)) * scale_factor_; //1 m/s

    simulation_pose.drive_direction = drive_direction;
    simulation_pose.v = this->speed_;
    simulation_pose.vdot = (this->speed_-old_speed)/((double)publish_milliseconds_/1000)*scale_factor_;//1 m/s^2

    simulation_pose.curv = simulation_pose.thetadot/std::max(0.01,simulation_pose.v)/scale_factor_;//1/m

    simulation_pose.vehicle_length = 0.49*scale_factor_;
    simulation_pose.vehicle_width = 0.18*scale_factor_;

    publisher_->publish(simulation_pose);
}