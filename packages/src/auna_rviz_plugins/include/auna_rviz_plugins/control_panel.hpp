#ifndef AUNA_RVIZ_PLUGINS_CONTROL_PANEL_HPP_
#define AUNA_RVIZ_PLUGINS_CONTROL_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "std_msgs/msg/bool.hpp"

#include <chrono>  // Include chrono for time points
#include <memory>  // Include memory for unique_ptr etc.
#include <thread>  // Include thread

// Forward declare Qt classes
QT_BEGIN_NAMESPACE
class QLineEdit;
class QPushButton;
class QComboBox;
class QLabel;
class QVBoxLayout;
class QTimer;
QT_END_NAMESPACE

// Include ROS message/service headers
#include "auna_msgs/srv/set_string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace auna_rviz_plugins
{

enum class SelectedSource { OFF = 0, CACC = 1, TELEOP = 2, NAV2 = 3 };

}  // namespace auna_rviz_plugins

Q_DECLARE_METATYPE(auna_rviz_plugins::SelectedSource)

namespace auna_rviz_plugins
{

class ControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ControlPanel(QWidget * parent = nullptr);
  virtual ~ControlPanel() override;

  virtual void load(const rviz_common::Config & config) override;
  virtual void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onNamespaceChanged(const QString & text);
  void onSourceSelected(int index);
  void onEmergencyStopClicked();
  void checkServiceAvailability();
  void updateStatusIndicators();

private:
  // --- UI Elements ---
  QVBoxLayout * layout_ = nullptr;
  QLineEdit * namespace_input_ = nullptr;
  QComboBox * source_selector_ = nullptr;
  QPushButton * emergency_stop_button_ = nullptr;
  QLabel * status_label_ = nullptr;
  QLabel * cacc_status_ = nullptr;
  QLabel * teleop_status_ = nullptr;
  QLabel * nav2_status_ = nullptr;

  // --- ROS Components ---
  rclcpp::Node::SharedPtr node_ = nullptr;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_ = nullptr;
  std::unique_ptr<std::thread> spinner_thread_ = nullptr;

  // --- State Variables ---
  std::string current_namespace_;
  SelectedSource selected_source_ = SelectedSource::OFF;
  bool estop_active_ = false;
  bool backend_node_ready_ = false;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr global_estop_publisher_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_estop_subscriber_;
  std::chrono::time_point<std::chrono::steady_clock> cacc_last_msg_time_;
  std::chrono::time_point<std::chrono::steady_clock> teleop_last_msg_time_;
  std::chrono::time_point<std::chrono::steady_clock> nav2_last_msg_time_;
  bool cacc_msg_received_ = false;
  bool teleop_msg_received_ = false;
  bool nav2_msg_received_ = false;

  // --- ROS Clients and Subscribers ---
  rclcpp::Client<auna_msgs::srv::SetString>::SharedPtr set_source_client_ = nullptr;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_estop_client_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cacc_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_subscriber_ = nullptr;

  // --- Timers ---
  QTimer * ui_update_timer_ = nullptr;
  QTimer * status_timer_ = nullptr;

  // --- Helper Methods ---
  void setupUI();
  void initializeROS();
  void updateUIStates();
  void updateROSSubscriptionsAndClients();
  void callSetSourceService(const std::string & source_name);
  void callSetEstopService(bool activate);
  void setStatusLED(QLabel * label, bool active);
  bool isMessageRecent(
    const std::chrono::time_point<std::chrono::steady_clock> & last_msg_time) const;
  void caccCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void teleopCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void nav2Callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void globalEstopCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace auna_rviz_plugins

#endif  // AUNA_RVIZ_PLUGINS_CONTROL_PANEL_HPP_