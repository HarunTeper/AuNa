#include "rclcpp/rclcpp.hpp"

#include <etsi_its_msgs_utils/cam_access.hpp>

#include "auna_msgs/srv/set_bool.hpp"
#include "auna_msgs/srv/set_float64.hpp"
#include "etsi_its_cam_msgs/msg/cam.hpp"
#include "geometry_msgs/msg/pose_array.hpp"  // Added include for PoseArray
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include <fstream>
#include <iomanip>  // for std::setprecision

// includes for tf matrix
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

struct Parameters
{
  double standstill_distance;
  double time_gap;
  double kp;
  double kd;
  double max_velocity;
  bool use_waypoints;
  int frequency;
  double target_velocity;
  double curvature_lookahead;
  double extra_distance;
};

class CaccController : public rclcpp::Node
{
public:
  CaccController();

private:
  etsi_its_cam_msgs::msg::CAM create_cam_debug_message();

  // node variables
  rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr sub_cam_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_waypoints_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr setup_timer_;

  // service for standstill_distance and time_gap
  rclcpp::Service<auna_msgs::srv::SetFloat64>::SharedPtr client_set_standstill_distance_;
  rclcpp::Service<auna_msgs::srv::SetFloat64>::SharedPtr client_set_time_gap_;

  // service for target_velocity and extra_distance
  rclcpp::Service<auna_msgs::srv::SetFloat64>::SharedPtr client_set_target_velocity_;
  rclcpp::Service<auna_msgs::srv::SetFloat64>::SharedPtr client_set_extra_distance_;

  // service for cacc_enable
  rclcpp::Service<auna_msgs::srv::SetBool>::SharedPtr client_set_cacc_enable_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_x_lookahead_point_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_y_lookahead_point_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_closest_pose_waypoint_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_waypoint_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_closest_cam_waypoint_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cacc_pose_;

  // pub_cam_ publisher for cam
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr pub_cam_;

  // variables for cam_callback
  double cam_x_;
  double cam_y_;
  double cam_velocity_;
  double cam_acceleration_;
  double cam_yaw_;
  double cam_yaw_rate_;
  double cam_curvature_;
  etsi_its_cam_msgs::msg::CAM::SharedPtr last_cam_msg_;
  double last_cam_velocity_;

  // variables for odom_callback
  double odom_x_;
  double odom_y_;
  double odom_yaw_;
  double odom_velocity_;
  double odom_acceleration_;
  double odom_yaw_rate_;
  double odom_curvature_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
  double last_odom_velocity_;

  // variables for pose_callback
  double pose_x_;
  double pose_y_;
  double pose_yaw_;

  tf2::Quaternion q_;
  tf2::Matrix3x3 m_;
  double roll_;
  double pitch_;
  double yaw_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_msg_;

  // variables for control
  double control_x_;
  double control_y_;
  double control_velocity_;
  double control_last_velocity_ = 0;
  double control_acceleration_;
  double control_yaw_;
  double control_yaw_rate_;
  double control_curvature_;

  // variables for waypoints
  std::vector<double> waypoints_x_;
  std::vector<double> waypoints_y_;
  std::vector<double> waypoints_yaw_;

  // auto mode
  bool auto_mode_;
  bool auto_mode_ready_;
  bool cacc_ready_;
  double target_velocity_;

  // ros2 service server for auto_mode
  rclcpp::Service<auna_msgs::srv::SetBool>::SharedPtr client_set_auto_mode_;

  // Flags to track first message reception
  bool first_cam_received_;
  bool first_odom_received_;
  bool first_pose_received_;

  // general functions
  void update_waypoint_following();
  void publish_waypoint_pose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & publisher,
    int waypoint_index);

  // callback functions
  void cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void waypoints_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void timer_callback();
  void setup_timer_callback();

  // service callback functions
  void set_standstill_distance(
    const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
    std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response);
  void set_time_gap(
    const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
    std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response);
  void set_cacc_enable(
    const std::shared_ptr<auna_msgs::srv::SetBool::Request> request,
    std::shared_ptr<auna_msgs::srv::SetBool::Response> response);
  void set_auto_mode(
    const std::shared_ptr<auna_msgs::srv::SetBool::Request> request,
    std::shared_ptr<auna_msgs::srv::SetBool::Response> response);
  void set_target_velocity(
    const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
    std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response);
  void set_extra_distance(
    const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
    std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response);

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  Parameters params_;

  double s_ = 0;
  double alpha_ = 0;
  double x_lookahead_point_ = 0;
  double y_lookahead_point_ = 0;
  double z_1_ = 0;
  double z_2_ = 0;
  double z_3_ = 0;
  double z_4_ = 0;
  double invGam_1_ = 0;
  double invGam_2_ = 0;
  double invGam_3_ = 0;
  double invGam_4_ = 0;
  double invGam_Det_ = 0;
  double inP_1_ = 0;
  double inP_2_ = 0;
  double a_ = 0;
  double w_ = 0;
  double dt_ = 0;
  rclcpp::Time last_time_;
  double v_ = 0;
  double last_velocity_ = 0;

  // Intermediate calculation values for logging/debugging
  double dbg_alpha_ = 0;
  double dbg_s_ = 0;
  double dbg_invGam_1_ = 0;
  double dbg_invGam_2_ = 0;
  double dbg_invGam_3_ = 0;
  double dbg_invGam_4_ = 0;
  double dbg_inP1_pos_err_ = 0;
  double dbg_inP1_vel_err_ = 0;
  double dbg_inP1_geom_vel_ = 0;
  double dbg_inP1_yaw_rate_ = 0;
  double dbg_inP2_pos_err_ = 0;
  double dbg_inP2_vel_err_ = 0;
  double dbg_inP2_geom_vel_ = 0;
  double dbg_inP2_yaw_rate_ = 0;

  // Data logging
  bool enable_data_logging_;
  std::string log_file_path_;
  std::ofstream log_file_;
  int log_counter_ = 0;
};