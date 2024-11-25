#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "auna_its_msgs/msg/cam.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "auna_msgs/srv/set_float64.hpp"
#include "auna_msgs/srv/set_bool.hpp"

//includes for tf matrix
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "rcl_interfaces/msg/set_parameters_result.hpp"


struct Parameters
{
    double standstill_distance;
    double time_gap;
    double kp;
    double kd;
    double max_velocity;
    bool use_waypoints;
    std::string waypoint_file;
    int frequency;
    double target_velocity;
    int curvature_lookahead;
    double extra_distance;
};

class CaccController : public rclcpp::Node
{
    public:
        CaccController();
    private:
        //node variables
        rclcpp::Subscription<auna_its_msgs::msg::CAM>::SharedPtr sub_cam_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
        rclcpp::Publisher<auna_its_msgs::msg::CAM>::SharedPtr pub_cam_debug_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr setup_timer_;

        //service for standstill_distance and time_gap
        rclcpp::Service<auna_msgs::srv::SetFloat64>::SharedPtr client_set_standstill_distance_;
        rclcpp::Service<auna_msgs::srv::SetFloat64>::SharedPtr client_set_time_gap_;

        //service for target_velocity and extra_distance
        rclcpp::Service<auna_msgs::srv::SetFloat64>::SharedPtr client_set_target_velocity_;
        rclcpp::Service<auna_msgs::srv::SetFloat64>::SharedPtr client_set_extra_distance_;
        
        //service for cacc_enable
        rclcpp::Service<auna_msgs::srv::SetBool>::SharedPtr client_set_cacc_enable_;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_x_lookahead_point_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_y_lookahead_point_;

        //pub_cam_ publisher for cam
        rclcpp::Publisher<auna_its_msgs::msg::CAM>::SharedPtr pub_cam_;

        //variables for cam_callback
        double cam_x_;
        double cam_y_;
        double cam_velocity_;
        double cam_acceleration_;
        double cam_yaw_;
        double cam_yaw_rate_;
        double cam_curvature_;
        auna_its_msgs::msg::CAM::SharedPtr last_cam_msg_;
        double last_cam_velocity_;

        //variables for odom_callback
        double odom_x_;
        double odom_y_;
        double odom_yaw_;
        double odom_velocity_;
        double odom_acceleration_;
        double odom_yaw_rate_;
        double odom_curvature_;
        nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
        double last_odom_velocity_;

        //variables for pose_callback
        double pose_x_;
        double pose_y_;
        double pose_yaw_;

        tf2::Quaternion q_;
        tf2::Matrix3x3 m_;
        double roll_;
        double pitch_;
        double yaw_;
        geometry_msgs::msg::PoseStamped::SharedPtr last_pose_msg_;

        //variables for waypoints
        std::vector<double> waypoints_x_;
        std::vector<double> waypoints_y_;
        std::vector<double> waypoints_yaw_;

        //auto mode
        bool auto_mode_;
        bool auto_mode_ready_;
        double target_velocity_;

        //ros2 service server for auto_mode
        rclcpp::Service<auna_msgs::srv::SetBool>::SharedPtr client_set_auto_mode_;

        // general functions
        void read_waypoints_from_csv();

        //callback functions
        void cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void timer_callback();
        void setup_timer_callback();

        //service callback functions
        void set_standstill_distance(const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
                                    std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response);
        void set_time_gap(const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
                        std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response);
        void set_cacc_enable(const std::shared_ptr<auna_msgs::srv::SetBool::Request> request,
                            std::shared_ptr<auna_msgs::srv::SetBool::Response> response);
        void set_auto_mode(const std::shared_ptr<auna_msgs::srv::SetBool::Request> request,
                            std::shared_ptr<auna_msgs::srv::SetBool::Response> response);
        void set_target_velocity(const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
                                std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response);
        void set_extra_distance(const std::shared_ptr<auna_msgs::srv::SetFloat64::Request> request,
                                std::shared_ptr<auna_msgs::srv::SetFloat64::Response> response);

        //dynamic parameters
        rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
        Parameters params_;

        //controller function variables
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
};