#ifndef COMMUNICATION_COSTMAP_HPP_
#define COMMUNICATION_COSTMAP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "auna_its_msgs/msg/cam.hpp"
#include "nav2_util/string_utils.hpp"

namespace auna_comm
{

class CommunicationCostmap : public nav2_costmap_2d::Layer
{
public:
  CommunicationCostmap();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

private:
  // Parameters
  int robot_index_;

  // Subscription for CAM messages
  rclcpp::Subscription<auna_its_msgs::msg::CAM>::SharedPtr cam_sub_;

  // Callback for CAM messages
  void cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg);

  //vector of CAM messages
  std::vector<auna_its_msgs::msg::CAM> cam_msgs_;

  // Costmap coordinates
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

};

}  // namespace auna_comm

#endif  // COMMUNICATION_COSTMAP_HPP_
