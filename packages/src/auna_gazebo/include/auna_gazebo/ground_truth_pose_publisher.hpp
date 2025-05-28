#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class GroundTruthPosePublisher : public rclcpp::Node
{
public:
  GroundTruthPosePublisher();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  std::string prefix_;
};