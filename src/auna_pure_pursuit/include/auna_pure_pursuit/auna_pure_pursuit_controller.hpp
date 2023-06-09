#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>


#include "rclcpp/rclcpp.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/buffer.h"
//include nav2 on_activate


class AuNaPurePursuitController : public nav2_core::Controller
{
    public:
        AuNaPurePursuitController() = default;
        ~AuNaPurePursuitController() override = default;

        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped & pose,
            const geometry_msgs::msg::Twist & velocity,
            nav2_core::GoalChecker * /*goal_checker*/) override;

        void setPlan(const nav_msgs::msg::Path & path) override;

        void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

    private:
        nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

        bool transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose, geometry_msgs::msg::PoseStamped & out_pose) const;

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        nav2_costmap_2d::Costmap2D * costmap_;
        rclcpp::Logger logger_ {rclcpp::get_logger("AuNaPurePursuitController")};

        nav_msgs::msg::Path global_plan_;
        double control_duration_;

        double desired_linear_velocity_;
        double lookahead_distance_;
        tf2::Duration transform_tolerance_;
        double goal_dist_tol_;

        double base_desired_linear_velocity_;

        //create global path pub
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
        //create carrot pub
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>> carrot_pub_;
        //create extended carrot pub
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>> extended_carrot_pub_;

};