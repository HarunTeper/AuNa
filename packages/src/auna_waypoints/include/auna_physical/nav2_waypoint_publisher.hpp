#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <vector>

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class Nav2WaypointPublisher : public rclcpp::Node
{
public:
  Nav2WaypointPublisher();

private:
  // create a namespace variable
  std::string namespace_;

  // create a buffer and listener for tf2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // create a timer and its callback
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  // publish waypoints
  void publish_waypoints();
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_publisher_;

  // action client and callbacks
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr client_ptr_;
  void goal_response_callback(GoalHandleNavigateThroughPoses::SharedPtr goal_handle);
  void feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
  void result_callback(const GoalHandleNavigateThroughPoses::WrappedResult & result);

  // lifecycle state client to ensure bt_navigator is ACTIVE before sending goals
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr bt_get_state_client_;
  bool is_bt_navigator_active();

  // waypoint data
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::string waypoint_file_;
  int current_pose_index_ = 0;
  int remaining_number_of_poses_ = 0;
  bool remaining_decreased_ = false;
  int number_of_waypoints_ = 20;

  // behavior flags
  bool wait_for_bt_active_ = false;
};