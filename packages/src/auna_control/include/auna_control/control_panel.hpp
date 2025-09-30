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

#ifndef AUNA_CONTROL__CONTROL_PANEL_HPP_
#define AUNA_CONTROL__CONTROL_PANEL_HPP_

#include "auna_control/control_panel_ros_interface.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <rviz_common/panel.hpp>

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

  void onEstopStatusUpdated(bool isActive, const QString & message);
  void onOdometryUpdated(double speed, double angular_vel, double x, double y, double z);
  void onImuUpdated(double ax, double ay, double az);
  void onCmdVelUpdated(double linear, double angular);
  void onInputSourcesUpdated(const QStringList & sources);
  void onSourceStatusUpdated(const QString & source);
  void onBackendReady(bool ready);

private:
  void setupUI();
  void setupMonitoringUI();
  void updateMonitoringDisplay();

  QVBoxLayout * layout_;
  QLineEdit * namespace_input_;
  QPushButton * emergency_stop_button_;
  QLabel * status_label_;
  QComboBox * source_combo_box_;
  QString last_known_source_;
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

  ControlPanelROSInterface * control_panel_ros_interface_;
  bool estop_active_;
};

}  // namespace auna_control

#endif  // AUNA_CONTROL__CONTROL_PANEL_HPP_
