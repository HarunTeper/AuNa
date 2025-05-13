#ifndef AUNA_RVIZ_PLUGINS_GLOBAL_EMERGENCY_STOP_PANEL_HPP_
#define AUNA_RVIZ_PLUGINS_GLOBAL_EMERGENCY_STOP_PANEL_HPP_

// Standard ROS 2 includes
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

// RViz common includes
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>

// Qt includes
#include <QtCore/QObject>
#include <QtCore/QStringList>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

// Standard C++ includes
#include <map>
#include <string>
#include <vector>

// Forward declarations - these need to be outside any namespace
class QLineEdit;
class QPushButton;
class QLabel;
class QVBoxLayout;

namespace auna_rviz_plugins
{

class GlobalEmergencyStopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  GlobalEmergencyStopPanel(QWidget * parent = nullptr);
  virtual ~GlobalEmergencyStopPanel();

  // Override load and save functions for panel persistence
  virtual void load(const rviz_common::Config & config) override;
  virtual void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onNamespacesChanged(const QString & text);
  void onGlobalEmergencyStopClicked();

private:
  // UI Elements
  QLineEdit * namespaces_input_;
  QPushButton * emergency_stop_button_;
  QLabel * namespaces_label_;
  QVBoxLayout * layout_;

  // ROS Elements
  rclcpp::Node::SharedPtr node_;
  // Map from namespace string to its cmd_vel publisher
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publisher_map_;

  // State Variables
  QStringList target_namespaces_;  // Store as QStringList for easier splitting/joining

  // Helper methods
  void initializeROS();
  void setupUI();
  void updatePublishers();
  geometry_msgs::msg::Twist createZeroTwist();
};

}  // namespace auna_rviz_plugins

#endif  // AUNA_RVIZ_PLUGINS_GLOBAL_EMERGENCY_STOP_PANEL_HPP_