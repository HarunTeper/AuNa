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


#ifndef AUNA_CONTROL__CONTROL_PANEL_ROS_INTERFACE_HPP_
#define AUNA_CONTROL__CONTROL_PANEL_ROS_INTERFACE_HPP_

#include <QObject>
#include <QString>
#include <rclcpp/rclcpp.hpp>

#include "auna_msgs/srv/set_string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <memory>
#include <thread>

using Trigger = std_srvs::srv::Trigger;

class ControlPanelROSInterface : public QObject
{
  Q_OBJECT

public:
  explicit ControlPanelROSInterface(QObject * parent = nullptr);
  ~ControlPanelROSInterface();

public Q_SLOTS:
  void setNamespace(const QString & ns);
  void triggerEstop(bool activate);
  void setCmdVelSource(const QString & source);
  void checkServices();
  void checkInputSources();
  void checkSourceStatus();

Q_SIGNALS:
  void estopStatusUpdated(bool isActive, const QString & message);
  void odometryUpdated(double speed, double angular_vel, double x, double y, double z);
  void imuUpdated(double ax, double ay, double az);
  void cmdVelUpdated(double linear, double angular);
  void inputSourcesUpdated(const QStringList & sources);
  void sourceStatusUpdated(const QString & source);
  void backendReady(bool ready);

private:
  void initializeROS();
  void createClientsAndSubscribers();
  void spin();

  void onEstopSetResponse(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);
  void onEstopStatusResponse(rclcpp::Client<Trigger>::SharedFuture future);
  void onInputSourcesResponse(rclcpp::Client<Trigger>::SharedFuture future);
  void onSourceStatusResponse(rclcpp::Client<Trigger>::SharedFuture future);
  void onSetSourceResponse(rclcpp::Client<auna_msgs::srv::SetString>::SharedFuture future);

  void onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onImuReceived(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr estop_status_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr estop_set_client_;
  rclcpp::Client<Trigger>::SharedPtr source_status_client_;
  rclcpp::Client<Trigger>::SharedPtr input_sources_client_;
  rclcpp::Client<auna_msgs::srv::SetString>::SharedPtr set_source_client_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<std::thread> spinner_thread_;

  std::string current_namespace_;
};

#endif  // AUNA_CONTROL__CONTROL_PANEL_ROS_INTERFACE_HPP_

