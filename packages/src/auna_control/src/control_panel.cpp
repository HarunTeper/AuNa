#include "auna_control/control_panel.hpp"

#include <QtCore/QTimer>
#include <QtCore/QVariant>
#include <QtGui/QPainter>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include "auna_msgs/srv/set_string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <functional>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;

namespace auna_control
{

ControlPanel::ControlPanel(QWidget * parent)
: Panel(parent), current_namespace_(""), estop_active_(false), backend_node_ready_(false)
{
  layout_ = new QVBoxLayout();
  setupUI();
  initializeROS();
}

ControlPanel::~ControlPanel()
{
  if (executor_) executor_->cancel();
  if (spinner_thread_ && spinner_thread_->joinable()) spinner_thread_->join();
}

void ControlPanel::setupUI()
{
  // Setup simplified UI (Namespace + E-Stop Button)
  QGroupBox * ns_group = new QGroupBox("Target Namespace");
  QVBoxLayout * ns_layout = new QVBoxLayout;
  namespace_input_ = new QLineEdit(QString::fromStdString(current_namespace_));
  ns_layout->addWidget(namespace_input_);
  ns_group->setLayout(ns_layout);
  layout_->addWidget(ns_group);

  emergency_stop_button_ = new QPushButton("Emergency STOP");
  emergency_stop_button_->setStyleSheet("background-color: red; color: white; font-weight: bold;");
  emergency_stop_button_->setCheckable(true);
  layout_->addWidget(emergency_stop_button_);

  status_label_ = new QLabel("Status: Initializing...");
  layout_->addWidget(status_label_);

  source_combo_box_ = new QComboBox();
  layout_->addWidget(source_combo_box_);
  // Only call setCmdVelSource if the user changed the combobox, not when updating programmatically
  connect(
    source_combo_box_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int index) { this->onSourceComboBoxChanged(index); });

  connect(namespace_input_, &QLineEdit::textChanged, this, &ControlPanel::onNamespaceChanged);
  connect(
    emergency_stop_button_, &QPushButton::clicked, this, &ControlPanel::onEmergencyStopClicked);

  layout_->addStretch(1);
  QTimer::singleShot(0, [this]() { namespace_input_->setFocus(); });
  this->setFocusPolicy(Qt::StrongFocus);
  // Add monitoring UI after existing elements
  setupMonitoringUI();
  setLayout(layout_);

  // layout_->addStretch(1);
  // QTimer::singleShot(0, [this]() { namespace_input_->setFocus(); });
}

void ControlPanel::initializeROS()
{
  node_ = std::make_shared<rclcpp::Node>("rviz_control_panel_node");

  estop_status_client_ = node_->create_client<Trigger>("/get_estop_status");
  estop_set_client_ = node_->create_client<std_srvs::srv::SetBool>("/trigger_emergency_stop");
  createComboBoxServiceClients();

  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spinner_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });

  status_timer_ = new QTimer();
  connect(status_timer_, &QTimer::timeout, this, &ControlPanel::checkServiceAvailability);
  status_timer_->start(1000);

  checkEstopServiceAvailability();
  checkComboBoxServiceAvailability();

  createMonitoringSubscribers();
}

void ControlPanel::createComboBoxServiceClients()
{
  source_status_client_ = node_->create_client<std_srvs::srv::Trigger>(
    current_namespace_.empty() ? "/get_source_status"
                               : "/" + current_namespace_ + "/get_source_status");
  std::string input_sources_service_name = current_namespace_.empty()
                                             ? "/get_input_sources"
                                             : "/" + current_namespace_ + "/get_input_sources";
  RCLCPP_INFO(
    node_->get_logger(), "Creating client for service: %s", input_sources_service_name.c_str());
  input_sources_client_ = node_->create_client<std_srvs::srv::Trigger>(input_sources_service_name);
  set_source_client_ = node_->create_client<auna_msgs::srv::SetString>(
    current_namespace_.empty() ? "/toggle_cmd_vel_source"
                               : "/" + current_namespace_ + "/toggle_cmd_vel_source");
}

void ControlPanel::onNamespaceChanged(const QString & text)
{
  current_namespace_ = text.toStdString();
  source_combo_box_->clear();
  createComboBoxServiceClients();
  checkEstopServiceAvailability();
  checkComboBoxServiceAvailability();
  createMonitoringSubscribers();
}

void ControlPanel::onEmergencyStopClicked()
{
  RCLCPP_INFO(rclcpp::get_logger("control_panel"), "onEmergencyStopClicked: Button pressed");
  if (
    !estop_status_client_ || !estop_status_client_->service_is_ready() || !estop_set_client_ ||
    !estop_set_client_->service_is_ready()) {
    RCLCPP_INFO(
      rclcpp::get_logger("control_panel"), "onEmergencyStopClicked: Service not available");
    QMessageBox::warning(this, "Error", "E-Stop service not available.");
    return;
  }

  // Phase 1: Before click (UI disables button, shows waiting)
  RCLCPP_INFO(
    rclcpp::get_logger("control_panel"),
    "onEmergencyStopClicked: Disabling button and updating status label");
  emergency_stop_button_->setEnabled(false);
  status_label_->setText("Status: Sending E-STOP request...");

  // Phase 2: During click (send service request)
  RCLCPP_INFO(
    rclcpp::get_logger("control_panel"),
    "onEmergencyStopClicked: Sending service request with data=%d", !estop_active_);
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !estop_active_;  // Toggle estop

  estop_set_client_->async_send_request(
    request, std::bind(&ControlPanel::onEstopSetResponse, this, std::placeholders::_1));
}

void ControlPanel::onEstopSetResponse(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result)
{
  RCLCPP_INFO(rclcpp::get_logger("control_panel"), "onEstopSetResponse: Service response received");
  auto response = result.get();
  bool success = response->success;
  QString msg = QString::fromStdString(response->message);
  QMetaObject::invokeMethod(
    this,
    [this, success, msg]() {
      RCLCPP_INFO(rclcpp::get_logger("control_panel"), "onEstopSetResponse: Updating UI");
      emergency_stop_button_->setEnabled(true);
      if (success) {
        status_label_->setText("Status: " + msg);
      } else {
        status_label_->setText("Status: Failed to set E-STOP: " + msg);
      }
      // Force a status refresh to update button state
      checkEstopServiceAvailability();
    },
    Qt::QueuedConnection);
}

void ControlPanel::updateUIStates()
{
  // Always enable the button unless a request is in progress
  emergency_stop_button_->setEnabled(true);
  if (!estop_status_client_ || !estop_status_client_->service_is_ready()) {
    status_label_->setText("Status: Waiting for backend...");
    emergency_stop_button_->setEnabled(false);
    return;
  }
  if (estop_active_) {
    emergency_stop_button_->setText("E-STOP ACTIVE (Click to Release)");
    emergency_stop_button_->setStyleSheet(
      "background-color: gray; color: white; font-weight: bold;");
    status_label_->setText("Status: E-STOP ACTIVE");
  } else {
    emergency_stop_button_->setText("Emergency STOP (Click to Activate)");
    emergency_stop_button_->setStyleSheet(
      "background-color: red; color: white; font-weight: bold;");
    status_label_->setText("Status: Ready");
  }
}

void ControlPanel::checkEstopServiceAvailability()
{
  backend_node_ready_ = (node_ != nullptr);
  // Query estop status
  if (estop_status_client_ && estop_status_client_->service_is_ready()) {
    auto request = std::make_shared<Trigger::Request>();
    estop_status_client_->async_send_request(
      request, [this](rclcpp::Client<Trigger>::SharedFuture result) {
        this->onEstopStatusResponse(result);
      });
  } else {
    updateUIStates();
  }
}

void ControlPanel::checkComboBoxServiceAvailability()
{
  // Query input sources and current source
  if (input_sources_client_) {
    RCLCPP_INFO(
      node_->get_logger(), "Checking for service: %s", input_sources_client_->get_service_name());
    if (input_sources_client_->service_is_ready()) {
      RCLCPP_INFO(node_->get_logger(), "Service is ready, sending request.");
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      input_sources_client_->async_send_request(
        req, std::bind(&ControlPanel::onInputSourcesResponse, this, std::placeholders::_1));
    } else {
      RCLCPP_WARN(node_->get_logger(), "Service not ready.");
    }
  }
  if (source_status_client_ && source_status_client_->service_is_ready()) {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    source_status_client_->async_send_request(
      req, std::bind(&ControlPanel::onSourceStatusResponse, this, std::placeholders::_1));
  }
}

void ControlPanel::checkServiceAvailability() { checkEstopServiceAvailability(); }

void ControlPanel::onEstopStatusResponse(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result)
{
  // Use the exact message to determine estop state
  const std::string & msg = result.get()->message;
  if (msg == "Emergency stop is ACTIVE") {
    estop_active_ = true;
  } else if (msg == "Emergency stop is INACTIVE") {
    estop_active_ = false;
  }  // else: leave unchanged
  QMetaObject::invokeMethod(
    this, [this]() { updateUIStates(); }, Qt::QueuedConnection);
}

void ControlPanel::onInputSourcesResponse(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result)
{
  RCLCPP_INFO(rclcpp::get_logger("control_panel"), "onInputSourcesResponse called");
  if (!result.get()->success) {
    RCLCPP_ERROR(
      rclcpp::get_logger("control_panel"), "Get input sources service failed: %s",
      result.get()->message.c_str());
    return;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("control_panel"), "Received sources: %s", result.get()->message.c_str());
  QStringList sources =
    QString::fromStdString(result.get()->message).split(",", Qt::SkipEmptyParts);
  source_combo_box_->clear();
  source_combo_box_->addItems(sources);
}

void ControlPanel::onSourceStatusResponse(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result)
{
  RCLCPP_INFO(rclcpp::get_logger("control_panel"), "onSourceStatusResponse called");
  QString current = QString::fromStdString(result.get()->message).section(": ", 1);
  int idx = source_combo_box_->findText(current);
  if (idx >= 0) {
    source_combo_box_->setCurrentIndex(idx);
  }
}

void ControlPanel::onSourceComboBoxChanged(int index)
{
  if (index >= 0 && index < source_combo_box_->count()) {
    QString selected = source_combo_box_->itemText(index);
    this->setCmdVelSource(selected);
  }
}

void ControlPanel::setCmdVelSource(const QString & source)
{
  RCLCPP_INFO(
    rclcpp::get_logger("control_panel"), "setCmdVelSource called with source: %s",
    source.toStdString().c_str());
  if (!set_source_client_) {
    RCLCPP_ERROR(rclcpp::get_logger("control_panel"), "set_source_client_ is not initialized");
    return;
  }
  auto req = std::make_shared<auna_msgs::srv::SetString::Request>();
  req->data = source.toStdString();
  set_source_client_->async_send_request(
    req, [this](rclcpp::Client<auna_msgs::srv::SetString>::SharedFuture) {
      checkServiceAvailability();
    });
}

void ControlPanel::load(const rviz_common::Config & config)
{
  // No-op for now
}

void ControlPanel::save(rviz_common::Config config) const
{
  // No-op for now
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

void ControlPanel::createMonitoringSubscribers()
{
  std::string ns_prefix = current_namespace_.empty() ? "" : "/" + current_namespace_;

  // Subscribe to odometry for position and velocity
  odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    ns_prefix + "/odom", 10,
    std::bind(&ControlPanel::onOdometryReceived, this, std::placeholders::_1));

  // Subscribe to cmd_vel to monitor commanded velocities (from multiplexer output)
  cmd_vel_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    ns_prefix + "/cmd_vel_twist", 10,
    std::bind(&ControlPanel::onCmdVelReceived, this, std::placeholders::_1));

  // Subscribe to IMU for acceleration (if available)
  imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    ns_prefix + "/imu", 10, std::bind(&ControlPanel::onImuReceived, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(), "Created monitoring subscribers for namespace: %s",
    current_namespace_.c_str());
}

void ControlPanel::onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract position
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_z_ = msg->pose.pose.position.z;

  // Extract linear velocity (speed)
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  double vz = msg->twist.twist.linear.z;
  current_speed_ = sqrt(vx * vx + vy * vy + vz * vz);

  // Extract angular velocity
  current_angular_vel_ = msg->twist.twist.angular.z * 180.0 / M_PI;
  ;

  // Update UI in main thread
  QMetaObject::invokeMethod(
    this, [this]() { updateMonitoringDisplay(); }, Qt::QueuedConnection);
}

void ControlPanel::onCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_linear_x_ = msg->linear.x;
  cmd_angular_z_ = msg->angular.z * 180.0 / M_PI;
  ;

  // Add debug logging to see if data is received
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000, "Received cmd_vel: linear=%.2f, angular=%.2f",
    cmd_linear_x_, cmd_angular_z_);

  // Update UI in main thread with proper Qt formatting
  QMetaObject::invokeMethod(
    this,
    [this]() {
      // Use proper Qt string formatting - NOT %.2f!
      cmd_vel_label_->setText(QString("%1, %2")
                                .arg(QString::number(cmd_linear_x_, 'f', 2))
                                .arg(QString::number(cmd_angular_z_, 'f', 2)));
    },
    Qt::QueuedConnection);
}
void ControlPanel::onImuReceived(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_accel_x_ = msg->linear_acceleration.x;
  current_accel_y_ = msg->linear_acceleration.y;
  current_accel_z_ = msg->linear_acceleration.z;

  // Update UI in main thread
  QMetaObject::invokeMethod(
    this,
    [this]() {
      acceleration_label_->setText(QString("%1, %2, %3")
                                     .arg(QString::number(current_accel_x_, 'f', 2))
                                     .arg(QString::number(current_accel_y_, 'f', 2))
                                     .arg(QString::number(current_accel_z_, 'f', 2)));
    },
    Qt::QueuedConnection);
}

void ControlPanel::updateMonitoringDisplay()
{
  // Use QString::number() for more reliable formatting
  speed_label_->setText(QString::number(current_speed_, 'f', 2));

  position_label_->setText(QString("%1, %2, %3")
                             .arg(QString::number(current_x_, 'f', 2))
                             .arg(QString::number(current_y_, 'f', 2))
                             .arg(QString::number(current_z_, 'f', 2)));

  acceleration_label_->setText(QString("%1, %2, %3")
                                 .arg(QString::number(current_accel_x_, 'f', 2))
                                 .arg(QString::number(current_accel_y_, 'f', 2))
                                 .arg(QString::number(current_accel_z_, 'f', 2)));

  angular_velocity_label_->setText(QString::number(current_angular_vel_, 'f', 2));
}

}  // namespace auna_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(auna_control::ControlPanel, rviz_common::Panel)