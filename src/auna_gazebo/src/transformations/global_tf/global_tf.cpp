#include "auna_gazebo/global_tf.hpp"

// Create a service callback to the Gazebo models to get the current robot names.
GlobalTF::GlobalTF() : Node("global_tf_node"), tf_broadcaster_(this)
{ 
    service_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),[this](){service_timer_callback();});
    modelClient_ = this->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
}



// Callback for the timer. Creates a service call to get the Gazebo model names.
void GlobalTF::service_timer_callback()
{
    auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
    while (!modelClient_->wait_for_service(std::chrono::seconds(5))) 
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto result = modelClient_->async_send_request(request,std::bind(&GlobalTF::model_srv_callback,this,std::placeholders::_1));
}

// Service callback of the Gazebo models.
void GlobalTF::model_srv_callback(const rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedFuture future)
{
    auto result = future.get();
    for(std::string model_name : result.get()->model_names)
    {
        // Only check for names that include robot
        if(model_name.find("robot") != std::string::npos)
        {
            // For each model name, check if already added. If not, add subscribers for local tf topics.
            if (std::find(robot_models_.begin(), robot_models_.end(), model_name) == robot_models_.end()) {
                robot_models_.push_back(model_name);

                tf_subscribers_.push_back(this->create_subscription<tf2_msgs::msg::TFMessage>("/"+model_name+"/tf", 2, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg){tf_callback(msg);}));
                tf_subscribers_.push_back(this->create_subscription<tf2_msgs::msg::TFMessage>("/"+model_name+"/tf_static", 2, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg){tf_callback(msg);}));
            }
        }
    }
}

// Callback for local tf topics. Publishes the transformations to the global tf topic.
void GlobalTF::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    for(const geometry_msgs::msg::TransformStamped &message: msg->transforms)
    {
        tf_broadcaster_.sendTransform(message);
    }
}
