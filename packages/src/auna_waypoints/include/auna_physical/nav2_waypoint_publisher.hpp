#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include <vector>

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class Nav2WaypointPublisher : public rclcpp::Node
{
public:
  Nav2WaypointPublisher();

private:
  // Parameters
  std::string namespace_;
  std::string waypoint_file_;

  // TF components (from original)
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Timer for publishing pose array and managing waypoint cycles
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  // Waypoint publishing
  void publish_waypoints();
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_publisher_;

  // Action client and callbacks
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
  void goal_response_callback(GoalHandleNavigateThroughPoses::SharedPtr goal_handle);
  void feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
  void result_callback(const GoalHandleNavigateThroughPoses::WrappedResult & result);

  // Waypoint data
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  int current_pose_index_ = 0;
  int remaining_number_of_poses_ = 0;
  int number_of_waypoints_ = 0;

  // Startup management
  bool action_server_available_ = false;
  int goal_rejection_count_ = 0;
  bool waiting_for_retry_ = false;
  static constexpr int MAX_GOAL_REJECTIONS = 5;
};