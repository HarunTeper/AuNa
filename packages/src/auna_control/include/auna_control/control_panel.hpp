#pragma once

#include <QtCore/QTimer>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "auna_msgs/srv/set_string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <memory>
#include <thread>

namespace auna_control
{

class ControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ControlPanel(QWidget * parent = nullptr);
  ~ControlPanel() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onNamespaceChanged(const QString & text);
  void onEmergencyStopClicked();
  void onSourceComboBoxChanged(int index);
  void updateUIStates();
  void checkServiceAvailability();
  void onEstopSetResponse(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result);
  void onEstopStatusResponse(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result);
  void onInputSourcesResponse(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result);
  void onSourceStatusResponse(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result);

private:
  void setupUI();
  void initializeROS();
  void createComboBoxServiceClients();
  void checkEstopServiceAvailability();
  void checkComboBoxServiceAvailability();
  void setCmdVelSource(const QString & source);

  // Monitoring UI
  void setupMonitoringUI();
  void createMonitoringSubscribers();
  void onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onImuReceived(const sensor_msgs::msg::Imu::SharedPtr msg);
  void updateMonitoringDisplay();

  QVBoxLayout * layout_;
  QLineEdit * namespace_input_;
  QPushButton * emergency_stop_button_;
  QLabel * status_label_;
  QComboBox * source_combo_box_;
  QTimer * status_timer_;

  // Monitoring UI elements
  QLabel * speed_label_;
  QLabel * position_label_;
  QLabel * acceleration_label_;
  QLabel * angular_velocity_label_;
  QLabel * cmd_vel_label_;

  // Monitoring data
  double current_speed_ = 0.0;
  double current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;
  double current_accel_x_ = 0.0, current_accel_y_ = 0.0, current_accel_z_ = 0.0;
  double current_angular_vel_ = 0.0;
  double cmd_linear_x_ = 0.0, cmd_angular_z_ = 0.0;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<std::thread> spinner_thread_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr estop_set_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr estop_status_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr source_status_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr input_sources_client_;
  rclcpp::Client<auna_msgs::srv::SetString>::SharedPtr set_source_client_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  std::string current_namespace_;
  bool estop_active_;
  bool backend_node_ready_;
};

}  // namespace auna_control