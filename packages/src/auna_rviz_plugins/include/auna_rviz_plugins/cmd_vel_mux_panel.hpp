#ifndef AUNA_RVIZ_PLUGINS_CMD_VEL_MUX_PANEL_HPP_
#define AUNA_RVIZ_PLUGINS_CMD_VEL_MUX_PANEL_HPP_

// Standard ROS 2 includes
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

// RViz common includes
#include <rviz_common/config.hpp>  // Added for load/save
#include <rviz_common/panel.hpp>

// Qt includes
#include <QtCore/QObject>  // Needed for Q_OBJECT macro and slots/signals
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>  // Keep for now
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>  // Base class for widgets

// Standard C++ includes
#include <map>
#include <string>
#include <vector>

namespace auna_rviz_plugins
{

// Forward declarations (optional but can speed up compilation)
class QLineEdit;
class QComboBox;
class QPushButton;
class QLabel;
class QVBoxLayout;

// Enum for command velocity sources
enum class SelectedSource { OFF, CACC, TELEOP, NAV2 };

class CmdVelMuxPanel : public rviz_common::Panel
{
  Q_OBJECT  // Macro for Qt's meta-object system (signals/slots)

    public : CmdVelMuxPanel(QWidget * parent = nullptr);
  virtual ~CmdVelMuxPanel();  // Virtual destructor is good practice

  // Override load and save functions for panel persistence
  virtual void load(const rviz_common::Config & amp; config) override;  // Added override specifier
  virtual void save(rviz_common::Config config) const override;         // Added override specifier

  // Use standard Qt slot declaration syntax
protected slots:
  void onNamespaceChanged(const QString & amp; text);
  void onSourceSelected(int index);  // QComboBox index

private:
  // UI Elements
  QLineEdit * namespace_input_;
  QComboBox * source_selector_;
  QLabel * namespace_label_;
  QLabel * source_label_;
  QVBoxLayout * layout_;

  // ROS Elements
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cacc_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_subscriber_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // State Variables
  std::string current_namespace_;
  SelectedSource current_source_ = SelectedSource::OFF;  // Initialize state
  geometry_msgs::msg::Twist latest_cacc_msg_;
  geometry_msgs::msg::Twist latest_teleop_msg_;
  geometry_msgs::msg::Twist latest_nav2_msg_;
  bool cacc_msg_received_ = false;
  bool teleop_msg_received_ = false;
  bool nav2_msg_received_ = false;

  // Helper methods
  void initializeROS();
  void setupUI();
  void updateSubscriptionsAndPublisher();  // Combined update logic
  void publishCommand();
  void caccCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  geometry_msgs::msg::Twist createZeroTwist();
};

}  // namespace auna_rviz_plugins

#endif  // AUNA_RVIZ_PLUGINS_CMD_VEL_MUX_PANEL_HPP_