#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class MQTTWaypointReceiver : public rclcpp::Node
{
    public:
        MQTTWaypointReceiver();
    private:

        //create a timer
        rclcpp::TimerBase::SharedPtr timer_;
        
        //create a timer callback
        void timer_callback();

        //create a client for the action
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr client_ptr_;

        //create a goal response callback
        void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr> future);

        //create a feedback callback
        void feedback_callback(
            GoalHandleNavigateThroughPoses::SharedPtr,
            const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);

        //create a result callback
        void result_callback(const GoalHandleNavigateThroughPoses::WrappedResult & result);

};