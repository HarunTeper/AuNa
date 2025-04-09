#include "auna_gazebo/global_tf.hpp"

// Create a service callback to the Gazebo models to get the current robot names.
GlobalTF::GlobalTF() : Node("global_tf_node"), tf_broadcaster_(this), static_tf_broadcaster_(this)
{
  service_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(100), [this]() { service_timer_callback(); });
  modelClient_ = this->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
}

// Callback for the timer. Creates a service call to get the Gazebo model names.
void GlobalTF::service_timer_callback()
{
  auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
  while (!modelClient_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  auto result = modelClient_->async_send_request(
    request, std::bind(&GlobalTF::model_srv_callback, this, std::placeholders::_1));
}

// Service callback of the Gazebo models.
void GlobalTF::model_srv_callback(
  const rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedFuture future)
{
  auto result = future.get();
  for (std::string model_name : result.get()->model_names) {
    // Only check for names that include robot
    if (model_name.find("robot") != std::string::npos) {
      // For each model name, check if already added. If not, add subscribers for local tf topics.
      if (
        std::find(robot_models_.begin(), robot_models_.end(), model_name) == robot_models_.end()) {
        robot_models_.push_back(model_name);

        // Use capturing lambda to pass the robot name to the callback
        tf_subscribers_.push_back(this->create_subscription<tf2_msgs::msg::TFMessage>(
          "/" + model_name + "/tf", 2,
          [this, robot_name = model_name](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            tf_callback(msg, robot_name, false);  // Dynamic transform
          }));
        // Define QoS profile for static transforms (compatible with transient_local)
        auto static_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        tf_subscribers_.push_back(this->create_subscription<tf2_msgs::msg::TFMessage>(
          "/" + model_name + "/tf_static", static_qos,
          [this, robot_name = model_name](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            tf_callback(msg, robot_name, true);  // Static transform
          }));
      }
    }
  }
}

// Callback for local tf topics. Publishes the transformations to the global tf topic.
void GlobalTF::tf_callback(
  const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string & robot_name, bool is_static)
{
  for (const geometry_msgs::msg::TransformStamped & message : msg->transforms) {
    // Create a modified transform with robot namespace
    // RCLCPP_INFO(
    //   this->get_logger(), "Original transform from %s (static: %s): %s -> %s",
    //   robot_name.c_str(), is_static ? "true" : "false", message.header.frame_id.c_str(),
    //   message.child_frame_id.c_str());

    geometry_msgs::msg::TransformStamped modified = message;

    std::string original_header = message.header.frame_id;
    std::string original_child = message.child_frame_id;

    // Case 1: map -> odom transform
    if (original_header == "map" && original_child == "odom") {
      modified.header.frame_id = "map";                // Keep map global
      modified.child_frame_id = robot_name + "/odom";  // Namespace odom
    }
    // Case 2: odom -> base_link transform
    else if (
      original_header == "odom" &&
      (original_child == "base_link" ||
       original_child == "base_footprint")) {           // Common base frame names
      modified.header.frame_id = robot_name + "/odom";  // Namespace odom
      modified.child_frame_id =
        robot_name + "/" + original_child;  // Namespace base_link/base_footprint
    }
    // Case 3: Other transforms (assume robot-internal)
    else {
      // Prefix header if not already prefixed
      if (modified.header.frame_id.find(robot_name + "/") != 0) {
        modified.header.frame_id = robot_name + "/" + modified.header.frame_id;
      }
      // Prefix child if not already prefixed
      if (modified.child_frame_id.find(robot_name + "/") != 0) {
        modified.child_frame_id = robot_name + "/" + modified.child_frame_id;
      }
    }

    // Log the transform we're republishing
    // RCLCPP_INFO(
    //   this->get_logger(), "Republishing transform (static: %s): %s -> %s (from robot %s)",
    //   is_static ? "true" : "false", modified.header.frame_id.c_str(),
    //   modified.child_frame_id.c_str(), robot_name.c_str());

    // Send the modified transform using the appropriate broadcaster
    if (is_static) {
      static_tf_broadcaster_.sendTransform(modified);
    } else {
      tf_broadcaster_.sendTransform(modified);
    }
  }
}
