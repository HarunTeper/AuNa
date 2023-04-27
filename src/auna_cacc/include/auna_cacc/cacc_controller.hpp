
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "auna_its_msgs/msg/cam.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

//includes for tf matrix
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"




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
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_standstill_distance_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_time_gap_;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_x_lookahead_point_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_y_lookahead_point_;

        //parameters
        double standstill_distance_;
        double time_gap_;
        double wheelbase_;
        double kp_;
        double kd_;
        double max_velocity_;

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

        //callback functions
        void cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void timer_callback();
        void standstill_distance_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void time_gap_callback(const std_msgs::msg::Float64::SharedPtr msg);

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