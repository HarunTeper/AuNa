#include "rclcpp/rclcpp.hpp"

#include "auna_its_msgs/msg/cam.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>

class CamCommunication : public rclcpp::Node
{
    public:
        CamCommunication();
    private:
        //subscriber and publisher, timer for cam messages
        rclcpp::Subscription<auna_its_msgs::msg::CAM>::SharedPtr cam_subscriber_;
        rclcpp::Publisher<auna_its_msgs::msg::CAM>::SharedPtr cam_filtered_publisher_;
        rclcpp::Publisher<auna_its_msgs::msg::CAM>::SharedPtr cam_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        //subscriber for pose and odom messages
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

        //callback functions
        void cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg);
        void timer_callback();
        void publish_cam_msg(std::string frame_id);

        //filter index variable
        int filter_index_ = 0;
        int robot_index_ = 0;

        //callback functions for pose and odom messages
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        //variables for pose and odom messages
        geometry_msgs::msg::PoseStamped pose_;
        nav_msgs::msg::Odometry odom_;

        //variables for pose and odom callbacks
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

        auna_its_msgs::msg::CAM last_cam_msg_;
        rclcpp::Time last_cam_msg_time_;
        


};