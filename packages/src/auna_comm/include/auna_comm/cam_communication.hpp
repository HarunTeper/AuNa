#ifndef CAM_COMMUNICATION_HPP_
#define CAM_COMMUNICATION_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <etsi_its_cam_msgs/msg/cam.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <fstream>
#include <iomanip>

class CamCommunication : public rclcpp::Node
{
public:
  CamCommunication();
  static constexpr std::chrono::milliseconds T_GenCamMin{100};    // 100ms = 10Hz max
  static constexpr std::chrono::milliseconds T_GenCamMax{1000};   // 1000ms = 1Hz min
  static constexpr std::chrono::milliseconds T_CheckCamGen{100};  // Check interval

private:
  // subscriber and publisher, timer for cam messages
  rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_subscriber_;
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  // subscriber for pose and odom messages
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  // callback functions
  void cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg);
  void timer_callback();
  void publish_cam_msg(const std::string & trigger);

  // filter index variable
  int filter_index_ = 0;
  int robot_index_ = 0;

  // callback functions for pose and odom messages
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // variables for pose and odom messages
  geometry_msgs::msg::PoseStamped pose_;
  nav_msgs::msg::Odometry odom_;

  // variables for pose and odom callbacks
  nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;

  float latitude_ = 0.0;
  float longitude_ = 0.0;
  float altitude_ = 0.0;
  float heading_ = 0.0;
  float drive_direction_ = 0.0;
  float speed_ = 0.0;
  float acceleration_ = 0.0;
  float yaw_rate_ = 0.0;
  float curvature_ = 0.0;
  float old_speed_ = 0.0;

  // etsi_its_cam_ts_msgs::msg::CAM last_cam_msg_; // No longer needed, publish new message every
  // time.
  rclcpp::Time last_cam_msg_time_;
  // Variables to store the last values, to check if a new message has to be sent
  float last_cam_msg_speed_ = 0.0;
  float last_cam_msg_latitude_ = 0.0;
  float last_cam_msg_longitude_ = 0.0;
  float last_cam_msg_heading_ = 0.0;

  double vehicle_length_;
  double vehicle_width_;

  // CAM message logging
  bool enable_cam_logging_;
  std::string cam_log_file_path_;
  std::ofstream cam_log_file_;
};
#endif  // CAM_COMMUNICATION_HPP_