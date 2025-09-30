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

#include "auna_control/control_panel_ros_interface.hpp"

#include <QStringList>
#include <QTimer>
#include <chrono>
#include <cmath>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ControlPanelROSInterface::ControlPanelROSInterface(QObject * parent)
: QObject(parent)
{
  initializeROS();
}

ControlPanelROSInterface::~ControlPanelROSInterface()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spinner_thread_ && spinner_thread_->joinable()) {
    spinner_thread_->join();
  }
}

void ControlPanelROSInterface::initializeROS()
{
  node_ = std::make_shared<rclcpp::Node>("rviz_control_panel_ros_interface");
  RCLCPP_DEBUG(
    node_->get_logger(),
    "Initializing ROS node and creating clients/subscribers");
  createClientsAndSubscribers();
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spinner_thread_ = std::make_unique<std::thread>([this]() {spin();});
}

void ControlPanelROSInterface::spin()
{
  if (executor_) {
    executor_->spin();
  }
}

void ControlPanelROSInterface::createClientsAndSubscribers()
{
  std::string ns_prefix =
    current_namespace_.empty() ? "" : "/" + current_namespace_;

  if (node_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Creating clients and subscribers for namespace: '%s'",
      ns_prefix.c_str());
  }

  estop_status_client_ = node_->create_client<Trigger>("/get_estop_status");
  estop_set_client_ =
    node_->create_client<std_srvs::srv::SetBool>("/trigger_emergency_stop");

  source_status_client_ =
    node_->create_client<Trigger>(ns_prefix + "/get_source_status");
  input_sources_client_ =
    node_->create_client<Trigger>(ns_prefix + "/get_input_sources");
  set_source_client_ = node_->create_client<auna_msgs::srv::SetString>(
    ns_prefix + "/toggle_cmd_vel_source");

  odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    ns_prefix + "/odom", 10,
    std::bind(
      &ControlPanelROSInterface::onOdometryReceived, this,
      std::placeholders::_1));
  cmd_vel_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    ns_prefix + "/cmd_vel_twist", 10,
    std::bind(
      &ControlPanelROSInterface::onCmdVelReceived, this,
      std::placeholders::_1));
  imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    ns_prefix + "/imu", 10,
    std::bind(
      &ControlPanelROSInterface::onImuReceived, this,
      std::placeholders::_1));
}

void ControlPanelROSInterface::setNamespace(const QString & ns)
{
  // Validate the namespace string to prevent crashes
  std::string ns_str = ns.toStdString();

  // Check for invalid characters that could cause ROS topic/service name issues
  if (ns_str.find(' ') != std::string::npos) {
    if (node_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Invalid namespace with spaces: '%s', ignoring",
        ns_str.c_str());
    }
    return;
  }

  current_namespace_ = ns_str;
  if (node_) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Setting namespace to: '%s'",
      current_namespace_.c_str());
  }
  createClientsAndSubscribers();
  checkServices();
  checkInputSources();
  checkSourceStatus();
}

void ControlPanelROSInterface::triggerEstop(bool activate)
{
  if (!estop_set_client_ || !estop_set_client_->service_is_ready()) {
    if (node_) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "E-Stop service not available when trying to %s E-Stop",
        activate ? "activate" : "deactivate");
    }
    emit estopStatusUpdated(false, "E-Stop service not available.");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = activate;
  if (node_) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Sending E-Stop %s request",
      activate ? "activate" : "deactivate");
  }
  estop_set_client_->async_send_request(
    request, std::bind(
      &ControlPanelROSInterface::onEstopSetResponse, this,
      std::placeholders::_1));
}

void ControlPanelROSInterface::setCmdVelSource(const QString & source)
{
  if (!set_source_client_ || !set_source_client_->service_is_ready()) {
    if (node_) {
      RCLCPP_DEBUG(node_->get_logger(), "Set source service not available");
    }
    return;
  }
  auto request = std::make_shared<auna_msgs::srv::SetString::Request>();
  request->data = source.toStdString();
  if (node_) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Setting cmd_vel source to: '%s'",
      request->data.c_str());
  }
  set_source_client_->async_send_request(
    request, std::bind(
      &ControlPanelROSInterface::onSetSourceResponse, this,
      std::placeholders::_1));
}

void ControlPanelROSInterface::checkServices()
{
  bool ready = estop_status_client_ && estop_status_client_->service_is_ready();
  if (node_) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Checking services. Backend ready: %s",
      ready ? "true" : "false");
  }
  emit backendReady(ready);

  if (ready) {
    auto request = std::make_shared<Trigger::Request>();
    estop_status_client_->async_send_request(
      request, std::bind(
        &ControlPanelROSInterface::onEstopStatusResponse,
        this, std::placeholders::_1));
  }
}

void ControlPanelROSInterface::checkInputSources()
{
  if (input_sources_client_ && input_sources_client_->service_is_ready()) {
    auto request = std::make_shared<Trigger::Request>();
    input_sources_client_->async_send_request(
      request, std::bind(
        &ControlPanelROSInterface::onInputSourcesResponse,
        this, std::placeholders::_1));
  }
}

void ControlPanelROSInterface::checkSourceStatus()
{
  if (source_status_client_ && source_status_client_->service_is_ready()) {
    auto request = std::make_shared<Trigger::Request>();
    source_status_client_->async_send_request(
      request, std::bind(
        &ControlPanelROSInterface::onSourceStatusResponse,
        this, std::placeholders::_1));
  }
}

void ControlPanelROSInterface::onEstopSetResponse(
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
{
  auto response = future.get();
  (void)response;
  if (node_) {
    RCLCPP_DEBUG(node_->get_logger(), "Received E-Stop set response");
  }
  checkServices();  // Re-check status after setting
}

void ControlPanelROSInterface::onEstopStatusResponse(
  rclcpp::Client<Trigger>::SharedFuture future)
{
  auto response = future.get();
  bool is_active = response->message == "Emergency stop is ACTIVE";
  if (node_) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Received E-Stop status: %s",
      response->message.c_str());
  }
  emit estopStatusUpdated(is_active, QString::fromStdString(response->message));
}

void ControlPanelROSInterface::onInputSourcesResponse(
  rclcpp::Client<Trigger>::SharedFuture future)
{
  auto response = future.get();
  if (response->success) {
    if (node_) {
      RCLCPP_DEBUG(
        node_->get_logger(), "Received input sources: %s",
        response->message.c_str());
    }
    QStringList sources = QString::fromStdString(response->message)
      .split(",", Qt::SkipEmptyParts);
    emit inputSourcesUpdated(sources);
  }
}

void ControlPanelROSInterface::onSourceStatusResponse(
  rclcpp::Client<Trigger>::SharedFuture future)
{
  auto response = future.get();
  if (response->success) {
    if (node_) {
      RCLCPP_DEBUG(
        node_->get_logger(), "Received source status: %s",
        response->message.c_str());
    }
    QString current =
      QString::fromStdString(response->message).section(": ", 1);
    emit sourceStatusUpdated(current);
  }
}

void ControlPanelROSInterface::onSetSourceResponse(
  rclcpp::Client<auna_msgs::srv::SetString>::SharedFuture future)
{
  // After setting source, check the updated status
  (void)future;
  if (node_) {
    RCLCPP_DEBUG(node_->get_logger(), "Received set source response");
  }
  // Check source status to get the updated active source
  checkSourceStatus();
}

void ControlPanelROSInterface::onOdometryReceived(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  double vz = msg->twist.twist.linear.z;
  double speed = sqrt(vx * vx + vy * vy + vz * vz);
  double angular_vel = msg->twist.twist.angular.z * 180.0 / M_PI;
  if (node_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Received odometry: speed=%.2f, angular_vel=%.2f, pos=(%.2f, "
      "%.2f, %.2f)",
      speed, angular_vel, msg->pose.pose.position.x,
      msg->pose.pose.position.y, msg->pose.pose.position.z);
  }
  emit odometryUpdated(speed, angular_vel, msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void ControlPanelROSInterface::onCmdVelReceived(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (node_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Received cmd_vel: linear=%.2f, angular=%.2f", msg->linear.x,
      msg->angular.z * 180.0 / M_PI);
  }
  emit cmdVelUpdated(msg->linear.x, msg->angular.z * 180.0 / M_PI);
}

void ControlPanelROSInterface::onImuReceived(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (node_) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Received IMU: ax=%.2f, ay=%.2f, az=%.2f",
      msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z);
  }
  emit imuUpdated(msg->linear_acceleration.x, msg->linear_acceleration.y,
    msg->linear_acceleration.z);
}
