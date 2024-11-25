#include "auna_comm/communication_costmap.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

int determineNamespaceIndex(const std::string& namespace_str)
{
  std::vector<std::string> tokens;

  // Split the input string by '/'
  size_t start = namespace_str.find_first_of('/');
  if (start != std::string::npos)
  {
    start++; // Skip the first '/'
    size_t end = namespace_str.find('/', start);
    std::string remaining_str = namespace_str.substr(start, end - start);

    // Look for the last '_' character in the remaining string
    size_t underscore_pos = remaining_str.find_last_of('_');
    if (underscore_pos != std::string::npos)
    {
      std::string index_str = remaining_str.substr(underscore_pos + 1);
      try
      {
        int index = std::stoi(index_str);
        return index;
      }
      catch (const std::invalid_argument& e)
      {
        // Ignore if the substring after '_' is not a valid integer
      }
    }
    else
    {
      // Extract trailing numbers at the end of the remaining string
      size_t i = remaining_str.length();
      while (i > 0 && std::isdigit(remaining_str[i - 1]))
      {
        i--;
      }
      if (i < remaining_str.length())
      {
        std::string index_str = remaining_str.substr(i);
        try
        {
          int index = std::stoi(index_str);
          return index;
        }
        catch (const std::invalid_argument& e)
        {
          // Ignore if the trailing substring is not a valid integer
        }
      }
    }
  }

  return 0; // Fallback value if no index is found or conversion fails
}

namespace auna_comm
{

CommunicationCostmap::CommunicationCostmap()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
CommunicationCostmap::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  declareParameter("robot_index", rclcpp::ParameterValue(0));
  node->get_parameter(name_ + "." + "robot_index", robot_index_);

  robot_index_=determineNamespaceIndex(node->get_namespace());

  RCLCPP_INFO(node->get_logger(), "CommunicationCostmap: robot_index: %d", robot_index_);

  cam_sub_ = node->create_subscription<auna_its_msgs::msg::CAM>("/cam", rclcpp::QoS(1),[this](const auna_its_msgs::msg::CAM::SharedPtr msg) {cam_callback(msg);});

  need_recalculation_ = false;
  current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
CommunicationCostmap::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
CommunicationCostmap::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "CommunicationCostmap::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
CommunicationCostmap::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Simply computing one-by-one cost per each cell
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = NO_INFORMATION;


      // for (auto object : cam_msgs_.objects) {
      //   double obj_x = object.pose.x;
      //   double obj_y = object.pose.y;
      //   double obj_size = object.size;

      //   if (i >= obj_x - obj_size && i <= obj_x + obj_size &&
      //       j >= obj_y - obj_size && j <= obj_y + obj_size) {
      //     // Object is within the cell, check its velocity
      //     if (object.velocity < 0.1) {
      //       // Object's velocity is less than 0.1m, mark the cell as an obstacle
      //       cost = LETHAL_OBSTACLE;
      //       break; // No need to check other objects
      //     }
      //   }
      // }

      // Check if the cell is within the received cam msg pose and size
      for(auto msg : cam_msgs_){
        double obj_x = msg.x+0.16;
        double obj_y = msg.y;
        double obj_size = msg.vehicle_length;

        if (i >= obj_x - obj_size && i <= obj_x + obj_size &&
            j >= obj_y - obj_size && j <= obj_y + obj_size) {
          // Object is within the cell, check its velocity
          if (msg.v < 0.1) {
            // Object's velocity is less than 0.1m, mark the cell as an obstacle
            cost = LETHAL_OBSTACLE;
            break; // No need to check other objects
          }
        }
      }

      master_array[index] = cost;
    }
  }
}

void CommunicationCostmap::cam_callback(const auna_its_msgs::msg::CAM::SharedPtr msg)
{
  // cast msg->robot_name to int and check if it is the same as robot_index_
  if(std::stoi(msg->robot_name) == robot_index_){
    for (uint i = 0; i < cam_msgs_.size(); i++) {
      if (cam_msgs_[i].robot_name == msg->robot_name) {
        //if it is, update the message
        cam_msgs_[i] = *msg;
        return;
      }
    }
    cam_msgs_.push_back(*msg);
  }

}  

} // namespace nav2_gradient_costmap_plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(auna_comm::CommunicationCostmap, nav2_costmap_2d::Layer)
