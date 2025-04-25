#ifndef AUNA_RVIZ_PLUGINS_CMD_VEL_MUX_PANEL_HPP_
#define AUNA_RVIZ_PLUGINS_CMD_VEL_MUX_PANEL_HPP_

// Standard ROS 2 includes
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

// RViz common includes
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>

// Qt includes
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QVariant>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>  // Add QPushButton include
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

// Standard C++ includes
#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace auna_rviz_plugins
{

// Enum for command velocity sources
enum class SelectedSource { OFF, CACC, TELEOP, NAV2 };

/**
 * @brief RViz panel for controlling command velocity multiplexing
 *
 * This panel allows users to select between different command velocity
 * sources and publishes them to a common output topic.
 */
class CmdVelMuxPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  CmdVelMuxPanel(QWidget * parent = nullptr);
  virtual ~CmdVelMuxPanel();

  // Override load and save functions for panel persistence
  virtual void load(const rviz_common::Config & config) override;
  virtual void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onNamespaceChanged(const QString & text);
  void onSourceSelected(int index);
  void updateStatusIndicators();
  void onActivateButtonClicked();
  void onOffButtonClicked();

private:
  // UI Elements
  QLineEdit * namespace_input_ = nullptr;
  QComboBox * source_selector_ = nullptr;
  QLabel * namespace_label_ = nullptr;
  QLabel * source_label_ = nullptr;
  QPushButton * activate_button_ = nullptr;
  QPushButton * off_button_ = nullptr;
  QVBoxLayout * layout_ = nullptr;
  QLabel * cacc_status_ = nullptr;
  QLabel * teleop_status_ = nullptr;
  QLabel * nav2_status_ = nullptr;
  QTimer * status_timer_ = nullptr;

  // ROS Elements
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cacc_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_subscriber_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<std::thread> spinner_thread_;

  // State Variables
  std::string current_namespace_;
  SelectedSource staged_source_ = SelectedSource::OFF;     // Renamed from current_source_
  SelectedSource activated_source_ = SelectedSource::OFF;  // New state for activated source
  geometry_msgs::msg::Twist latest_cacc_msg_;
  geometry_msgs::msg::Twist latest_teleop_msg_;
  geometry_msgs::msg::Twist latest_nav2_msg_;
  bool cacc_msg_received_ = false;
  bool teleop_msg_received_ = false;
  bool nav2_msg_received_ = false;
  std::chrono::time_point<std::chrono::steady_clock> cacc_last_msg_time_;
  std::chrono::time_point<std::chrono::steady_clock> teleop_last_msg_time_;
  std::chrono::time_point<std::chrono::steady_clock> nav2_last_msg_time_;

  // Helper methods
  void initializeROS();
  void setupUI();
  void updateSubscriptionsAndPublisher();
  void publishCommand();
  void caccCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  geometry_msgs::msg::Twist createZeroTwist();
  void setStatusLED(QLabel * label, bool active);

  /**
   * @brief Check if a message from the given source is recent
   * @return true if a message has been received within the last second
   */
  bool isMessageRecent(
    const std::chrono::time_point<std::chrono::steady_clock> & last_msg_time) const;
};

}  // namespace auna_rviz_plugins

#endif  // AUNA_RVIZ_PLUGINS_CMD_VEL_MUX_PANEL_HPP_