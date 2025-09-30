// Copyright 2025 Harun Teper
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "auna_control/control_panel.hpp"

#include <QtCore/QRegExp>
#include <QtCore/QTimer>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QWidget>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <rclcpp/rclcpp.hpp>

namespace auna_control
{

ControlPanel::ControlPanel(QWidget * parent)
: Panel(parent), estop_active_(false)
{
  layout_ = new QVBoxLayout();
  setupUI();

  control_panel_ros_interface_ = new ControlPanelROSInterface(this);
  RCLCPP_DEBUG(rclcpp::get_logger("ControlPanel"), "ControlPanel constructed");
  connect(
    namespace_input_, &QLineEdit::textChanged, this,
    &ControlPanel::onNamespaceChanged);
  connect(
    emergency_stop_button_, &QPushButton::clicked, this,
    &ControlPanel::onEmergencyStopClicked);
  connect(
    source_combo_box_,
    QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    &ControlPanel::onSourceComboBoxChanged);

  connect(
    control_panel_ros_interface_,
    &ControlPanelROSInterface::estopStatusUpdated, this,
    &ControlPanel::onEstopStatusUpdated);
  connect(
    control_panel_ros_interface_,
    &ControlPanelROSInterface::odometryUpdated, this,
    &ControlPanel::onOdometryUpdated);
  connect(
    control_panel_ros_interface_, &ControlPanelROSInterface::imuUpdated,
    this, &ControlPanel::onImuUpdated);
  connect(
    control_panel_ros_interface_,
    &ControlPanelROSInterface::cmdVelUpdated, this,
    &ControlPanel::onCmdVelUpdated);
  connect(
    control_panel_ros_interface_,
    &ControlPanelROSInterface::inputSourcesUpdated, this,
    &ControlPanel::onInputSourcesUpdated);
  connect(
    control_panel_ros_interface_,
    &ControlPanelROSInterface::sourceStatusUpdated, this,
    &ControlPanel::onSourceStatusUpdated);
  connect(
    control_panel_ros_interface_, &ControlPanelROSInterface::backendReady,
    this, &ControlPanel::onBackendReady);

  status_timer_ = new QTimer(this);
  connect(
    status_timer_, &QTimer::timeout, control_panel_ros_interface_,
    &ControlPanelROSInterface::checkServices);
  status_timer_->start(250);
}

ControlPanel::~ControlPanel() {}

void ControlPanel::setupUI()
{
  QGroupBox * ns_group = new QGroupBox("Target Namespace");
  QVBoxLayout * ns_layout = new QVBoxLayout;
  ns_group->setLayout(ns_layout);
  layout_->addWidget(ns_group);

  namespace_input_ = new QLineEdit("");
  namespace_input_->setToolTip(
    "Enter a valid ROS namespace (alphanumeric, underscore, and forward "
    "slash only)");
  ns_layout->addWidget(namespace_input_);

  emergency_stop_button_ = new QPushButton("Emergency STOP");
  emergency_stop_button_->setStyleSheet(
    "background-color: red; color: white; font-weight: bold;");
  emergency_stop_button_->setCheckable(true);
  layout_->addWidget(emergency_stop_button_);

  status_label_ = new QLabel("Status: Initializing...");
  layout_->addWidget(status_label_);

  source_combo_box_ = new QComboBox();
  source_combo_box_->addItem("Querying...");
  layout_->addWidget(source_combo_box_);

  layout_->addStretch(1);
  QTimer::singleShot(0, [this]() {namespace_input_->setFocus();});
  this->setFocusPolicy(Qt::StrongFocus);
  setupMonitoringUI();
  setLayout(layout_);
}

void ControlPanel::onNamespaceChanged(const QString & text)
{
  // Validate namespace - ROS names cannot contain spaces or other invalid
  // characters
  QString cleaned_text = text;
  // Remove spaces and other invalid characters
  cleaned_text.replace(QRegExp("[^a-zA-Z0-9_/]"), "");

  // If the text was modified, update the input field
  if (cleaned_text != text) {
    namespace_input_->blockSignals(true);
    namespace_input_->setText(cleaned_text);
    namespace_input_->blockSignals(false);
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("ControlPanel"), "Namespace changed to: %s",
    cleaned_text.toStdString().c_str());
  last_known_source_ = "";
  source_combo_box_->clear();
  source_combo_box_->addItem("Querying...");
  control_panel_ros_interface_->setNamespace(cleaned_text);
}

void ControlPanel::onEmergencyStopClicked()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ControlPanel"),
    "Emergency stop button clicked. Current estop_active_: %d",
    estop_active_);
  control_panel_ros_interface_->triggerEstop(!estop_active_);
  emergency_stop_button_->setEnabled(false);
  status_label_->setText("Status: Sending E-STOP request...");
}

void ControlPanel::onSourceComboBoxChanged(int index)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ControlPanel"),
    "Source combo box changed. Index: %d", index);
  if (index >= 0) {
    control_panel_ros_interface_->setCmdVelSource(
      source_combo_box_->itemText(index));
  }
}

void ControlPanel::onEstopStatusUpdated(bool isActive, const QString & message)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ControlPanel"),
    "E-Stop status updated: %d, message: %s", isActive,
    message.toStdString().c_str());
  estop_active_ = isActive;
  emergency_stop_button_->setEnabled(true);
  status_label_->setText("Status: " + message);
  if (estop_active_) {
    emergency_stop_button_->setText("E-STOP ACTIVE (Click to Release)");
    emergency_stop_button_->setStyleSheet(
      "background-color: gray; color: white; font-weight: bold;");
  } else {
    emergency_stop_button_->setText("Emergency STOP (Click to Activate)");
    emergency_stop_button_->setStyleSheet(
      "background-color: red; color: white; font-weight: bold;");
  }
}

void ControlPanel::onOdometryUpdated(
  double speed, double angular_vel, double x,
  double y, double z)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ControlPanel"),
    "Odometry updated: speed=%.2f, angular_vel=%.2f, pos=(%.2f, %.2f, %.2f)",
    speed, angular_vel, x, y, z);
  current_speed_ = speed;
  current_angular_vel_ = angular_vel;
  current_x_ = x;
  current_y_ = y;
  current_z_ = z;
  updateMonitoringDisplay();
}

void ControlPanel::onImuUpdated(double ax, double ay, double az)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ControlPanel"),
    "IMU updated: ax=%.2f, ay=%.2f, az=%.2f", ax, ay, az);
  current_accel_x_ = ax;
  current_accel_y_ = ay;
  current_accel_z_ = az;
  updateMonitoringDisplay();
}

void ControlPanel::onCmdVelUpdated(double linear, double angular)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ControlPanel"),
    "CmdVel updated: linear=%.2f, angular=%.2f", linear, angular);
  cmd_linear_x_ = linear;
  cmd_angular_z_ = angular;
  updateMonitoringDisplay();
}

void ControlPanel::onInputSourcesUpdated(const QStringList & sources)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ControlPanel"), "Input sources updated: %s",
    sources.join(",").toStdString().c_str());
  source_combo_box_->blockSignals(true);
  source_combo_box_->clear();
  source_combo_box_->addItems(sources);
  int idx = source_combo_box_->findText(last_known_source_);
  if (idx >= 0) {
    source_combo_box_->setCurrentIndex(idx);
  }
  source_combo_box_->blockSignals(false);
}

void ControlPanel::onSourceStatusUpdated(const QString & source)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ControlPanel"), "Source status updated: %s",
    source.toStdString().c_str());
  last_known_source_ = source;
  int idx = source_combo_box_->findText(source);
  if (idx >= 0) {
    source_combo_box_->blockSignals(true);
    source_combo_box_->setCurrentIndex(idx);
    source_combo_box_->blockSignals(false);
  }
}

void ControlPanel::onBackendReady(bool ready)
{
  RCLCPP_DEBUG(rclcpp::get_logger("ControlPanel"), "Backend ready: %d", ready);
  emergency_stop_button_->setEnabled(ready);
  if (!ready) {
    status_label_->setText("Status: Waiting for backend...");
  }
}

void ControlPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  QString ns;
  if (config.mapGetString("namespace", &ns)) {
    namespace_input_->setText(ns);
  }
}

void ControlPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("namespace", namespace_input_->text());
}

void ControlPanel::setupMonitoringUI()
{
  // Create monitoring group
  QGroupBox * monitoring_group = new QGroupBox("Robot Monitoring");
  QGridLayout * monitoring_layout = new QGridLayout;

  // Speed monitoring
  QLabel * speed_title = new QLabel("Speed (m/s):");
  speed_label_ = new QLabel("0.00");
  speed_label_->setStyleSheet("font-weight: bold; color: blue;");
  monitoring_layout->addWidget(speed_title, 0, 0);
  monitoring_layout->addWidget(speed_label_, 0, 1);

  // Position monitoring
  QLabel * position_title = new QLabel("Position (x, y, z):");
  position_label_ = new QLabel("0.00, 0.00, 0.00");
  position_label_->setStyleSheet("font-weight: bold; color: green;");
  monitoring_layout->addWidget(position_title, 1, 0);
  monitoring_layout->addWidget(position_label_, 1, 1);

  // Acceleration monitoring
  QLabel * accel_title = new QLabel("Acceleration (m/s²):");
  acceleration_label_ = new QLabel("0.00, 0.00, 0.00");
  acceleration_label_->setStyleSheet("font-weight: bold; color: orange;");
  monitoring_layout->addWidget(accel_title, 2, 0);
  monitoring_layout->addWidget(acceleration_label_, 2, 1);

  // Angular velocity monitoring - updated label to show degrees
  QLabel * angular_title = new QLabel("Angular Vel (deg/s):");
  angular_velocity_label_ = new QLabel("0.00");
  angular_velocity_label_->setStyleSheet("font-weight: bold; color: purple;");
  monitoring_layout->addWidget(angular_title, 3, 0);
  monitoring_layout->addWidget(angular_velocity_label_, 3, 1);

  // Command velocity monitoring - updated label to show degrees for angular
  QLabel * cmd_vel_title = new QLabel("Cmd Vel (m/s, deg/s):");
  cmd_vel_label_ = new QLabel("0.00, 0.00");
  cmd_vel_label_->setStyleSheet("font-weight: bold; color: red;");
  monitoring_layout->addWidget(cmd_vel_title, 4, 0);
  monitoring_layout->addWidget(cmd_vel_label_, 4, 1);

  monitoring_group->setLayout(monitoring_layout);
  layout_->addWidget(monitoring_group);
}

void ControlPanel::updateMonitoringDisplay()
{
  // Use QString::number() for more reliable formatting
  speed_label_->setText(QString::number(current_speed_, 'f', 2));

  position_label_->setText(
    QString("%1, %2, %3")
    .arg(QString::number(current_x_, 'f', 2))
    .arg(QString::number(current_y_, 'f', 2))
    .arg(QString::number(current_z_, 'f', 2)));

  acceleration_label_->setText(
    QString("%1, %2, %3")
    .arg(QString::number(current_accel_x_, 'f', 2))
    .arg(QString::number(current_accel_y_, 'f', 2))
    .arg(QString::number(current_accel_z_, 'f', 2)));

  angular_velocity_label_->setText(
    QString::number(current_angular_vel_, 'f', 2));

  cmd_vel_label_->setText(
    QString("%1, %2")
    .arg(QString::number(cmd_linear_x_, 'f', 2))
    .arg(QString::number(cmd_angular_z_, 'f', 2)));
}

}  // namespace auna_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(auna_control::ControlPanel, rviz_common::Panel)
