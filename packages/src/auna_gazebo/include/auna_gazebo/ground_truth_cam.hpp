#include "rclcpp/rclcpp.hpp"

#include <etsi_its_msgs_utils/cam_access.hpp>  // access functions

#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "std_msgs/msg/header.hpp"
#include <etsi_its_cam_msgs/msg/cam.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>

class GroundTruthCam : public rclcpp::Node
{
public:
  GroundTruthCam(std::string name);

private:
  void service_timer_callback();
  void model_srv_callback(
    const rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future);

  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr modelClient_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr service_timer_;

  std::string name_;
  double speed_;
  int publish_milliseconds_ = 100;
  double scale_factor_ = 10;
};