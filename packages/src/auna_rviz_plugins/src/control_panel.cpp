#include "auna_rviz_plugins/control_panel.hpp"

// Include the generated service header explicitly
#include "auna_msgs/srv/set_string.hpp"
#include "std_srvs/srv/set_bool.hpp"

// Explicitly include Qt headers used in this file
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
#include <QtWidgets/QWidget>  // Include QWidget for parent pointers

#include <geometry_msgs/msg/twist.hpp>  // For subscribing to cmd_vel topics

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

// Use chrono literals (e.g., 500ms)
using namespace std::chrono_literals;

namespace auna_rviz_plugins
{

// Constructor
auna_rviz_plugins::ControlPanel::ControlPanel(QWidget * parent)
: rviz_common::Panel(parent),             // Initialize base class
  current_namespace_("robot1"),           // Initialize default namespace
  selected_source_(SelectedSource::OFF),  // Initialize selected source
  estop_active_(false),                   // Initialize E-stop state
  backend_node_ready_(false),             // Initialize backend readiness
  cacc_msg_received_(false),              // Initialize status flags
  teleop_msg_received_(false),
  nav2_msg_received_(false)
{
  // Create layout *before* setting it on the panel
  layout_ = new QVBoxLayout(this);  // Pass 'this' as parent

  // Setup UI elements and add them to layout_
  setupUI();

  // Set the layout *after* it's populated
  // setLayout(layout_); // layout_ is already set on 'this' in its constructor

  // Initialize ROS components *after* UI is fully set up
  initializeROS();
}

// Destructor
auna_rviz_plugins::ControlPanel::~ControlPanel()
{
  // Clean up ROS resources
  if (executor_) {
    executor_->cancel();
  }
  if (spinner_thread_ && spinner_thread_->joinable()) {
    spinner_thread_->join();
  }
  // Qt objects with 'this' as parent are automatically deleted
}

// Setup the UI elements and add them to the main layout_
void auna_rviz_plugins::ControlPanel::setupUI()
{
  // Namespace Group
  QGroupBox * namespace_group = new QGroupBox("Target Namespace", this);  // Parent = this
  QVBoxLayout * namespace_layout = new QVBoxLayout;
  namespace_input_ =
    new QLineEdit(QString::fromStdString(current_namespace_), this);  // Parent = this
  namespace_layout->addWidget(namespace_input_);
  namespace_group->setLayout(namespace_layout);
  layout_->addWidget(namespace_group);

  // Source Selection Group
  QGroupBox * source_group = new QGroupBox("Command Velocity Source", this);  // Parent = this
  QHBoxLayout * source_layout = new QHBoxLayout;
  source_selector_ = new QComboBox(this);  // Parent = this
  source_selector_->addItem("OFF", QVariant::fromValue(SelectedSource::OFF));
  source_selector_->addItem("CACC", QVariant::fromValue(SelectedSource::CACC));
  source_selector_->addItem("Teleop", QVariant::fromValue(SelectedSource::TELEOP));
  source_selector_->addItem("Nav2", QVariant::fromValue(SelectedSource::NAV2));
  source_layout->addWidget(source_selector_);
  source_group->setLayout(source_layout);
  layout_->addWidget(source_group);

  // Emergency Stop Button
  emergency_stop_button_ = new QPushButton("Emergency STOP", this);  // Parent = this
  emergency_stop_button_->setStyleSheet("background-color: red; color: white; font-weight: bold;");
  emergency_stop_button_->setCheckable(true);
  emergency_stop_button_->setChecked(estop_active_);
  layout_->addWidget(emergency_stop_button_);

  // Status indicators Group
  QGroupBox * status_group = new QGroupBox("Detected Sources", this);  // Parent = this
  QGridLayout * status_layout = new QGridLayout;

  // CACC status
  QLabel * cacc_label = new QLabel("CACC:", this);  // Parent = this
  cacc_status_ = new QLabel(this);                  // Parent = this
  cacc_status_->setFixedSize(20, 20);
  setStatusLED(cacc_status_, false);
  status_layout->addWidget(cacc_label, 0, 0);
  status_layout->addWidget(cacc_status_, 0, 1);

  // Teleop status
  QLabel * teleop_label = new QLabel("Teleop:", this);  // Parent = this
  teleop_status_ = new QLabel(this);                    // Parent = this
  teleop_status_->setFixedSize(20, 20);
  setStatusLED(teleop_status_, false);
  status_layout->addWidget(teleop_label, 1, 0);
  status_layout->addWidget(teleop_status_, 1, 1);

  // Nav2 status
  QLabel * nav2_label = new QLabel("Navigation:", this);  // Parent = this
  nav2_status_ = new QLabel(this);                        // Parent = this
  nav2_status_->setFixedSize(20, 20);
  setStatusLED(nav2_status_, false);
  status_layout->addWidget(nav2_label, 2, 0);
  status_layout->addWidget(nav2_status_, 2, 1);

  status_group->setLayout(status_layout);
  layout_->addWidget(status_group);

  // Status Label (for overall status)
  status_label_ = new QLabel("Status: Initializing...", this);  // Parent = this
  layout_->addWidget(status_label_);

  // Add stretch to push elements to the top
  layout_->addStretch(1);

  // Connect signals to slots
  connect(
    namespace_input_, &QLineEdit::textChanged, this,
    &auna_rviz_plugins::ControlPanel::onNamespaceChanged);
  connect(
    source_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    &auna_rviz_plugins::ControlPanel::onSourceSelected);
  connect(
    emergency_stop_button_, &QPushButton::clicked, this,
    &auna_rviz_plugins::ControlPanel::onEmergencyStopClicked);

  // Initially disable controls until ROS is ready
  updateUIStates();
}

// Initialize ROS components
void auna_rviz_plugins::ControlPanel::initializeROS()
{
  node_ = std::make_shared<rclcpp::Node>("rviz_control_panel_node");

  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spinner_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });

  auto now = std::chrono::steady_clock::now();
  cacc_last_msg_time_ = now;
  teleop_last_msg_time_ = now;
  nav2_last_msg_time_ = now;

  // Create timer for updating status indicators
  status_timer_ = new QTimer(this);  // Parent = this
  connect(
    status_timer_, &QTimer::timeout, this,
    &auna_rviz_plugins::ControlPanel::updateStatusIndicators);
  status_timer_->start(500ms);

  // Create timer to periodically check service availability
  ui_update_timer_ = new QTimer(this);  // Parent = this
  connect(
    ui_update_timer_, &QTimer::timeout, this,
    &auna_rviz_plugins::ControlPanel::checkServiceAvailability);
  ui_update_timer_->start(1000ms);

  updateROSSubscriptionsAndClients();
  checkServiceAvailability();  // Initial check
}

// Update ROS clients and subscribers based on the current namespace
void auna_rviz_plugins::ControlPanel::updateROSSubscriptionsAndClients()
{
  set_source_client_.reset();
  set_estop_client_.reset();
  cacc_subscriber_.reset();
  teleop_subscriber_.reset();
  nav2_subscriber_.reset();

  cacc_msg_received_ = false;
  teleop_msg_received_ = false;
  nav2_msg_received_ = false;
  updateStatusIndicators();

  if (current_namespace_.empty()) {
    RCLCPP_WARN(
      node_->get_logger(), "Namespace is empty. Clients and subscribers will not be created.");
    backend_node_ready_ = false;
    updateUIStates();
    return;
  }

  // Construct namespaced service names
  std::string set_source_service_name = "/" + current_namespace_ + "/set_cmd_vel_source";
  std::string set_estop_service_name = "/" + current_namespace_ + "/trigger_emergency_stop";

  set_source_client_ = node_->create_client<auna_msgs::srv::SetString>(set_source_service_name);
  set_estop_client_ = node_->create_client<std_srvs::srv::SetBool>(set_estop_service_name);
  RCLCPP_INFO(node_->get_logger(), "Using SetSource service: %s", set_source_service_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "Using SetEstop service: %s", set_estop_service_name.c_str());

  // Construct namespaced topic names for status monitoring (matching multiplexer inputs)
  std::string cacc_topic = "/" + current_namespace_ + "/cmd_vel_cacc";
  std::string teleop_topic = "/" + current_namespace_ + "/cmd_vel_teleop";
  std::string nav2_topic = "/" + current_namespace_ + "/cmd_vel_nav2";

  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // Create subscribers for status monitoring
  cacc_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cacc_topic, qos_profile,
    std::bind(&auna_rviz_plugins::ControlPanel::caccCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Monitoring CACC status on: %s", cacc_topic.c_str());

  teleop_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    teleop_topic, qos_profile,
    std::bind(&auna_rviz_plugins::ControlPanel::teleopCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Monitoring Teleop status on: %s", teleop_topic.c_str());

  nav2_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    nav2_topic, qos_profile,
    std::bind(&auna_rviz_plugins::ControlPanel::nav2Callback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Monitoring Nav2 status on: %s", nav2_topic.c_str());

  checkServiceAvailability();  // Re-check after creating clients
}

// Slot for namespace change
void auna_rviz_plugins::ControlPanel::onNamespaceChanged(const QString & text)
{
  std::string new_namespace = text.toStdString();
  if (new_namespace != current_namespace_) {
    current_namespace_ = new_namespace;
    RCLCPP_INFO(node_->get_logger(), "Namespace changed to: %s", current_namespace_.c_str());
    updateROSSubscriptionsAndClients();
    updateUIStates();
  }
}

// Slot for source selection change
void auna_rviz_plugins::ControlPanel::onSourceSelected(int index)
{
  if (index < 0 || !backend_node_ready_) return;

  QVariant data = source_selector_->itemData(index);
  selected_source_ = data.value<SelectedSource>();

  std::string source_name;
  // Map the selected enum back to the capitalized string expected by the backend service
  switch (selected_source_) {
    case SelectedSource::OFF:
      source_name = "OFF";
      break;
    case SelectedSource::CACC:
      source_name = "CACC";
      break;
    case SelectedSource::TELEOP:
      source_name = "TELEOP";
      break;
    case SelectedSource::NAV2:
      source_name = "NAV2";
      break;
    default:
      RCLCPP_WARN(node_->get_logger(), "Unknown source selected in dropdown.");
      return;
  }

  RCLCPP_INFO(node_->get_logger(), "UI: Source selected: %s", source_name.c_str());
  callSetSourceService(source_name);
}

// Slot for emergency stop button click
void auna_rviz_plugins::ControlPanel::onEmergencyStopClicked()
{
  if (!backend_node_ready_) {
    emergency_stop_button_->setChecked(!emergency_stop_button_->isChecked());
    QMessageBox::warning(this, "Error", "E-Stop service not available for the current namespace.");
    return;
  }

  estop_active_ = emergency_stop_button_->isChecked();
  RCLCPP_INFO(
    node_->get_logger(), "UI: E-Stop button clicked. Requesting state: %s",
    estop_active_ ? "ACTIVE" : "INACTIVE");

  callSetEstopService(estop_active_);
  updateUIStates();
}

void auna_rviz_plugins::ControlPanel::updateUIStates()
{
  if (estop_active_) {
    emergency_stop_button_->setStyleSheet(
      "background-color: gray; color: white; font-weight: bold;");
    emergency_stop_button_->setText("E-STOP ACTIVE (Click to Release)");
  } else {
    emergency_stop_button_->setStyleSheet(
      "background-color: red; color: white; font-weight: bold;");
    emergency_stop_button_->setText("Emergency STOP");
  }

  bool controls_enabled = backend_node_ready_ && !estop_active_;
  namespace_input_->setEnabled(true);
  source_selector_->setEnabled(controls_enabled);
  emergency_stop_button_->setEnabled(backend_node_ready_);

  if (current_namespace_.empty()) {
    status_label_->setText("Status: Enter Namespace");
  } else if (!backend_node_ready_) {
    status_label_->setText(QString("Status: Waiting for services in /%1...")
                             .arg(QString::fromStdString(current_namespace_)));
  } else if (estop_active_) {
    status_label_->setText("Status: E-STOP ACTIVE");
  } else {
    QVariant data = source_selector_->itemData(source_selector_->currentIndex());
    SelectedSource current_selection = data.value<SelectedSource>();
    std::string source_name = "OFF";
    switch (current_selection) {
      case SelectedSource::CACC:
        source_name = "CACC";
        break;
      case SelectedSource::TELEOP:
        source_name = "TELEOP";
        break;
      case SelectedSource::NAV2:
        source_name = "NAV2";
        break;
      default:
        break;
    }
    status_label_->setText(
      QString("Status: Ready (Source: %1)").arg(QString::fromStdString(source_name)));
  }
  // LEDs are updated by their own timer
}

// --- Service Call Implementations ---

void auna_rviz_plugins::ControlPanel::callSetSourceService(const std::string & source_name)
{
  if (!set_source_client_ || !set_source_client_->service_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "SetSource service client not available or not ready.");
    QMessageBox::warning(
      this, "Error", "Set Source service not available for the current namespace.");
    return;
  }
  auto request = std::make_shared<auna_msgs::srv::SetString::Request>();
  request->data = source_name;

  auto callback = [this,
                   source_name](rclcpp::Client<auna_msgs::srv::SetString>::SharedFuture future) {
    try {
      auto result = future.get();  // Get the result object
      if (result->success) {       // Access fields with '->'
        RCLCPP_INFO(
          this->node_->get_logger(), "Service call successful: %s", result->message.c_str());
        this->status_label_->setText(
          QString("Status: %1").arg(QString::fromStdString(result->message)));
      } else {
        RCLCPP_ERROR(this->node_->get_logger(), "Service call failed: %s", result->message.c_str());
        QMessageBox::warning(
          this, "Service Call Failed",
          QString("Failed to set source to %1: %2")
            .arg(QString::fromStdString(source_name))
            .arg(QString::fromStdString(result->message)));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->node_->get_logger(), "SetSource service call exception: %s", e.what());
      QMessageBox::critical(
        this, "Service Call Error", "Exception while calling SetSource service.");
    }
    this->checkServiceAvailability();
  };

  set_source_client_->async_send_request(request, callback);
  RCLCPP_DEBUG(node_->get_logger(), "Sent SetSource request: %s", source_name.c_str());
}

void auna_rviz_plugins::ControlPanel::callSetEstopService(bool activate)
{
  if (!set_estop_client_ || !set_estop_client_->service_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "SetEstop service client not available or not ready.");
    QMessageBox::warning(
      this, "Error", "Set E-Stop service not available for the current namespace.");
    emergency_stop_button_->setChecked(!activate);
    estop_active_ = !activate;
    updateUIStates();
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = activate;

  auto callback = [this, activate](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    bool call_success = false;
    bool service_reported_success = false;
    std::string message = "Service call failed";
    try {
      auto result = future.get();  // Get the result object
      call_success = true;         // If future.get() didn't throw, the call itself succeeded
      service_reported_success = result->success;  // Access fields with '->'
      message = result->message;
      if (service_reported_success) {
        RCLCPP_INFO(this->node_->get_logger(), "Service call successful: %s", message.c_str());
        this->status_label_->setText(QString("Status: %1").arg(QString::fromStdString(message)));
      } else {
        RCLCPP_ERROR(this->node_->get_logger(), "Service call failed: %s", message.c_str());
        QMessageBox::warning(
          this, "Service Call Failed",
          QString("Failed to set E-Stop state: %1").arg(QString::fromStdString(message)));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->node_->get_logger(), "SetEstop service call exception: %s", e.what());
      QMessageBox::critical(
        this, "Service Call Error", "Exception while calling SetEstop service.");
      message = "Service call exception";
    }

    // If the call failed OR the service reported failure, revert the state
    if (!call_success || !service_reported_success) {
      this->estop_active_ = !activate;  // Revert internal state
      this->emergency_stop_button_->blockSignals(true);
      this->emergency_stop_button_->setChecked(this->estop_active_);
      this->emergency_stop_button_->blockSignals(false);
    }

    this->updateUIStates();  // Update UI based on final state
  };

  set_estop_client_->async_send_request(request, callback);
  RCLCPP_DEBUG(node_->get_logger(), "Sent SetEstop request: %s", activate ? "true" : "false");
}

// Check if backend services are available
void auna_rviz_plugins::ControlPanel::checkServiceAvailability()
{
  if (!rclcpp::ok() || !node_) {
    backend_node_ready_ = false;
    if (ui_update_timer_) ui_update_timer_->stop();
    if (status_timer_) status_timer_->stop();
    updateUIStates();
    return;
  }

  bool source_ready =
    set_source_client_ && set_source_client_->wait_for_service(0s);  // Non-blocking check
  bool estop_ready =
    set_estop_client_ && set_estop_client_->wait_for_service(0s);  // Non-blocking check

  bool previously_ready = backend_node_ready_;
  backend_node_ready_ = source_ready && estop_ready;

  if (backend_node_ready_ != previously_ready) {
    RCLCPP_INFO(
      node_->get_logger(), "Backend services for namespace \'%s\' are %s",
      current_namespace_.c_str(), backend_node_ready_ ? "READY" : "NOT READY");
    updateUIStates();
  }
}

// --- Status Indicator Logic ---

void auna_rviz_plugins::ControlPanel::setStatusLED(QLabel * label, bool active)
{
  if (!label) return;

  QPixmap pixmap(16, 16);
  pixmap.fill(Qt::transparent);

  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing);

  QRadialGradient gradient(8, 8, 8);
  if (active) {
    gradient.setColorAt(0, Qt::green);
    gradient.setColorAt(1, QColor(0, 100, 0));
  } else {
    gradient.setColorAt(0, Qt::red);
    gradient.setColorAt(1, QColor(100, 0, 0));
  }

  painter.setBrush(gradient);
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(0, 0, 16, 16);

  label->setPixmap(pixmap);
}

bool auna_rviz_plugins::ControlPanel::isMessageRecent(
  const std::chrono::time_point<std::chrono::steady_clock> & last_msg_time) const
{
  return (std::chrono::steady_clock::now() - last_msg_time) < 1s;
}

void auna_rviz_plugins::ControlPanel::updateStatusIndicators()
{
  bool cacc_active = cacc_msg_received_ && isMessageRecent(cacc_last_msg_time_);
  bool teleop_active = teleop_msg_received_ && isMessageRecent(teleop_last_msg_time_);
  bool nav2_active = nav2_msg_received_ && isMessageRecent(nav2_last_msg_time_);

  setStatusLED(cacc_status_, cacc_active);
  setStatusLED(teleop_status_, teleop_active);
  setStatusLED(nav2_status_, nav2_active);
}

// --- Subscriber Callbacks ---

void auna_rviz_plugins::ControlPanel::caccCallback(
  const geometry_msgs::msg::Twist::ConstSharedPtr /*msg*/)
{
  cacc_msg_received_ = true;
  cacc_last_msg_time_ = std::chrono::steady_clock::now();
}

void auna_rviz_plugins::ControlPanel::teleopCallback(
  const geometry_msgs::msg::Twist::ConstSharedPtr /*msg*/)
{
  teleop_msg_received_ = true;
  teleop_last_msg_time_ = std::chrono::steady_clock::now();
}

void auna_rviz_plugins::ControlPanel::nav2Callback(
  const geometry_msgs::msg::Twist::ConstSharedPtr /*msg*/)
{
  nav2_msg_received_ = true;
  nav2_last_msg_time_ = std::chrono::steady_clock::now();
}

// --- Load/Save Configuration ---

void auna_rviz_plugins::ControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  // Load namespace from configuration if available
  QString ns;
  if (config.mapGetString("Namespace", &ns)) {
    if (!ns.isEmpty()) {
      RCLCPP_INFO(node_->get_logger(), "Loading saved namespace: %s", ns.toStdString().c_str());
      namespace_input_->blockSignals(true);
      namespace_input_->setText(ns);
      namespace_input_->blockSignals(false);
      onNamespaceChanged(ns);  // Update internal state and ROS clients/subs
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "No namespace configuration found, using default");
  }
  // Note: Source selection and E-stop state are intentionally not restored
  // as they should be determined by the current state of the robot
}

void auna_rviz_plugins::ControlPanel::save(rviz_common::Config config) const
{
  // First save our parent's state
  rviz_common::Panel::save(config);

  // Save our namespace configuration
  const QString ns = namespace_input_->text();
  config.mapSetValue("Namespace", ns);
  RCLCPP_INFO(node_->get_logger(), "Saving namespace configuration: %s", ns.toStdString().c_str());

  // Note: We don't save source or E-stop state as these are operational settings
  // that should not persist between sessions for safety reasons
}

}  // namespace auna_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(auna_rviz_plugins::ControlPanel, rviz_common::Panel)