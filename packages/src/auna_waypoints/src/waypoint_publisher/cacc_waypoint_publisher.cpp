#include "rclcpp/qos.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <sstream>

class CaccWaypointPublisher : public rclcpp::Node
{
public:
  CaccWaypointPublisher() : Node("cacc_waypoint_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/cacc/waypoints", rclcpp::QoS(1).transient_local());
    this->declare_parameter<std::string>("waypoint_file", "");
    this->get_parameter<std::string>("waypoint_file", waypoint_file_);

    read_waypoints();
  }

private:
  void read_waypoints()
  {
    std::ifstream file(waypoint_file_, std::ifstream::in);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open waypoint file: %s", waypoint_file_.c_str());
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string x, y;
      std::getline(iss, x, ',');
      std::getline(iss, y, ',');

      geometry_msgs::msg::Pose pose;
      pose.position.x = std::stod(x);
      pose.position.y = std::stod(y);
      waypoints_.poses.push_back(pose);
    }
    file.close();

    // Calculate orientation for each waypoint
    for (size_t i = 0; i < waypoints_.poses.size(); ++i) {
      double next_x = (i + 1 < waypoints_.poses.size()) ? waypoints_.poses[i + 1].position.x
                                                        : waypoints_.poses[0].position.x;
      double next_y = (i + 1 < waypoints_.poses.size()) ? waypoints_.poses[i + 1].position.y
                                                        : waypoints_.poses[0].position.y;
      double prev_x =
        (i > 0) ? waypoints_.poses[i - 1].position.x : waypoints_.poses.back().position.x;
      double prev_y =
        (i > 0) ? waypoints_.poses[i - 1].position.y : waypoints_.poses.back().position.y;

      double angle = atan2(next_y - prev_y, next_x - prev_x);
      tf2::Quaternion q;
      q.setRPY(0, 0, angle);
      waypoints_.poses[i].orientation = tf2::toMsg(q);
    }
  }

  void timer_callback()
  {
    waypoints_.header.stamp = this->get_clock()->now();
    waypoints_.header.frame_id = "map";
    publisher_->publish(waypoints_);
    timer_->cancel();
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string waypoint_file_;
  geometry_msgs::msg::PoseArray waypoints_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaccWaypointPublisher>());
  rclcpp::shutdown();
  return 0;
}