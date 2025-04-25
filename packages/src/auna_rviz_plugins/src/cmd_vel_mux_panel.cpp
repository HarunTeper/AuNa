#include "auna_rviz_plugins/cmd_vel_mux_panel.hpp"

#include <chrono>
#include <functional>

// Qt includes
#include <QtCore/QVariant>
#include <QtGui/QPainter>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>  // Add include
#include <QtWidgets/QVBoxLayout>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>

#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

namespace auna_rviz_plugins
{

CmdVelMuxPanel::CmdVelMuxPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  setupUI();
  initializeROS();
}

// Destructor
CmdVelMuxPanel::~CmdVelMuxPanel()
{
  // Stop the executor and spinner thread properly
  if (executor_) {
    executor_->cancel();
  }
  if (spinner_thread_ && spinner_thread_->joinable()) {
    spinner_thread_->join();
  }
}

// Setup UI elements
void CmdVelMuxPanel::setupUI()
{
  layout_ = new QVBoxLayout(this);

  // Namespace Input
  QGroupBox * namespace_group = new QGroupBox("Robot Namespace", this);
  QVBoxLayout * namespace_layout = new QVBoxLayout;
  namespace_input_ = new QLineEdit(this);
  namespace_input_->setPlaceholderText("e.g., robot1");
  connect(namespace_input_, &QLineEdit::textChanged, this, &CmdVelMuxPanel::onNamespaceChanged);
  namespace_layout->addWidget(namespace_input_);
  namespace_group->setLayout(namespace_layout);

  // Source Selector
  QGroupBox * source_group = new QGroupBox("Command Source", this);
  QVBoxLayout * source_layout = new QVBoxLayout;
  source_selector_ = new QComboBox(this);
  source_selector_->addItem("Off");
  source_selector_->addItem("CACC");
  source_selector_->addItem("Teleop");
  source_selector_->addItem("Navigation");
  connect(
    source_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    &CmdVelMuxPanel::onSourceSelected);
  source_layout->addWidget(source_selector_);
  source_group->setLayout(source_layout);

  // Buttons Layout (Activate and Off)
  QHBoxLayout * button_layout = new QHBoxLayout;
  activate_button_ = new QPushButton("Activate Selected Source", this);
  connect(activate_button_, &QPushButton::clicked, this, &CmdVelMuxPanel::onActivateButtonClicked);
  off_button_ = new QPushButton("Force Off", this);
  connect(off_button_, &QPushButton::clicked, this, &CmdVelMuxPanel::onOffButtonClicked);
  button_layout->addWidget(activate_button_);
  button_layout->addWidget(off_button_);

  // Status indicators
  QGroupBox * status_group = new QGroupBox("Status", this);
  QGridLayout * status_layout = new QGridLayout;

  // CACC status
  QLabel * cacc_label = new QLabel("CACC:", this);
  cacc_status_ = new QLabel(this);
  cacc_status_->setFixedSize(20, 20);
  setStatusLED(cacc_status_, false);
  status_layout->addWidget(cacc_label, 0, 0);
  status_layout->addWidget(cacc_status_, 0, 1);

  // Teleop status
  QLabel * teleop_label = new QLabel("Teleop:", this);
  teleop_status_ = new QLabel(this);
  teleop_status_->setFixedSize(20, 20);
  setStatusLED(teleop_status_, false);
  status_layout->addWidget(teleop_label, 1, 0);
  status_layout->addWidget(teleop_status_, 1, 1);

  // Nav2 status
  QLabel * nav2_label = new QLabel("Navigation:", this);
  nav2_status_ = new QLabel(this);
  nav2_status_->setFixedSize(20, 20);
  setStatusLED(nav2_status_, false);
  status_layout->addWidget(nav2_label, 2, 0);
  status_layout->addWidget(nav2_status_, 2, 1);

  status_group->setLayout(status_layout);

  // Add widgets to layout
  layout_->addWidget(namespace_group);
  layout_->addWidget(source_group);
  layout_->addLayout(button_layout);
  layout_->addWidget(status_group);
  layout_->addStretch();

  // Create timer for updating status indicators
  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, &CmdVelMuxPanel::updateStatusIndicators);
  status_timer_->start(500);

  setLayout(layout_);
}

void CmdVelMuxPanel::setStatusLED(QLabel * label, bool active)
{
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

bool CmdVelMuxPanel::isMessageRecent(
  const std::chrono::time_point<std::chrono::steady_clock> & last_msg_time) const
{
  return (std::chrono::steady_clock::now() - last_msg_time) < std::chrono::seconds(1);
}

void CmdVelMuxPanel::updateStatusIndicators()
{
  bool cacc_active = cacc_msg_received_ && isMessageRecent(cacc_last_msg_time_);
  bool teleop_active = teleop_msg_received_ && isMessageRecent(teleop_last_msg_time_);
  bool nav2_active = nav2_msg_received_ && isMessageRecent(nav2_last_msg_time_);

  setStatusLED(cacc_status_, cacc_active);
  setStatusLED(teleop_status_, teleop_active);
  setStatusLED(nav2_status_, nav2_active);
}

// Initialize ROS node, timer
void CmdVelMuxPanel::initializeROS()
{
  node_ = rclcpp::Node::make_shared("cmd_vel_mux_panel_node");
  staged_source_ = SelectedSource::OFF;     // Initialize staged source
  activated_source_ = SelectedSource::OFF;  // Initialize activated source
  current_namespace_ = "";

  // Create executor and spin node in separate thread
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  spinner_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });

  // Initialize timestamps
  auto now = std::chrono::steady_clock::now();
  cacc_last_msg_time_ = now;
  teleop_last_msg_time_ = now;
  nav2_last_msg_time_ = now;

  publish_timer_ =
    node_->create_wall_timer(100ms, std::bind(&CmdVelMuxPanel::publishCommand, this));

  updateSubscriptionsAndPublisher();
}

// Slot for namespace changes
void CmdVelMuxPanel::onNamespaceChanged(const QString & text)
{
  current_namespace_ = text.toStdString();
  RCLCPP_INFO(node_->get_logger(), "Namespace changed to: %s", current_namespace_.c_str());
  updateSubscriptionsAndPublisher();
  cacc_msg_received_ = false;
  teleop_msg_received_ = false;
  nav2_msg_received_ = false;

  // Reset sources on namespace change
  staged_source_ = SelectedSource::OFF;
  activated_source_ = SelectedSource::OFF;
  // Set index without emitting signal to avoid recursive calls if setText triggers this
  source_selector_->blockSignals(true);
  source_selector_->setCurrentIndex(0);  // Update UI to "Off"
  source_selector_->blockSignals(false);
  RCLCPP_INFO(node_->get_logger(), "Sources reset to OFF due to namespace change.");
}

// Slot for source selection changes (staging)
void CmdVelMuxPanel::onSourceSelected(int index)
{
  SelectedSource previously_staged = staged_source_;
  if (index >= 0) {
    switch (index) {
      case 0:
        staged_source_ = SelectedSource::OFF;
        // If "Off" is selected, deactivate immediately
        if (activated_source_ != SelectedSource::OFF) {
          activated_source_ = SelectedSource::OFF;
          RCLCPP_INFO(node_->get_logger(), "Source explicitly set to OFF. Deactivating.");
        }
        break;
      case 1:
        staged_source_ = SelectedSource::CACC;
        break;
      case 2:
        staged_source_ = SelectedSource::TELEOP;
        break;
      case 3:
        staged_source_ = SelectedSource::NAV2;
        break;
      default:
        staged_source_ = SelectedSource::OFF;
        break;
    }
    if (staged_source_ != previously_staged) {
      RCLCPP_INFO(node_->get_logger(), "Source staged: %d", static_cast<int>(staged_source_));
    }
  }
}

// Slot for activating the staged source
void CmdVelMuxPanel::onActivateButtonClicked()
{
  if (activated_source_ != staged_source_) {
    activated_source_ = staged_source_;
    const char * source_name = "UNKNOWN";
    switch (activated_source_) {
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
    }
    RCLCPP_INFO(
      node_->get_logger(), "Activated source: %s (%d)", source_name,
      static_cast<int>(activated_source_));
  } else {
    RCLCPP_DEBUG(
      node_->get_logger(), "Source %d already active.", static_cast<int>(activated_source_));
  }
}

// Slot for the Off button
void CmdVelMuxPanel::onOffButtonClicked()
{
  if (activated_source_ != SelectedSource::OFF || staged_source_ != SelectedSource::OFF) {
    RCLCPP_INFO(node_->get_logger(), "Force Off button clicked. Deactivating.");
    staged_source_ = SelectedSource::OFF;
    activated_source_ = SelectedSource::OFF;

    // Update the ComboBox UI to reflect the change
    source_selector_->blockSignals(true);  // Prevent onSourceSelected from triggering
    source_selector_->setCurrentIndex(0);  // 0 is the index for "Off"
    source_selector_->blockSignals(false);
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "Source is already OFF.");
  }
}

// Update ROS subscribers and publisher based on the current namespace
void CmdVelMuxPanel::updateSubscriptionsAndPublisher()
{
  cmd_vel_publisher_.reset();
  cacc_subscriber_.reset();
  teleop_subscriber_.reset();
  nav2_subscriber_.reset();

  if (current_namespace_.empty()) {
    RCLCPP_WARN(
      node_->get_logger(), "Namespace is empty. Publisher and subscribers will not be created.");
    return;
  }

  std::string cmd_vel_topic = "/" + current_namespace_ + "/cmd_vel";
  std::string cacc_topic = "/" + current_namespace_ + "/cmd_vel_cacc";
  std::string teleop_topic = "/" + current_namespace_ + "/cmd_vel_teleop";
  std::string nav2_topic = "/" + current_namespace_ + "/cmd_vel_nav2";

  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  cmd_vel_publisher_ =
    node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, qos_profile);
  RCLCPP_INFO(node_->get_logger(), "Publishing to: %s", cmd_vel_topic.c_str());

  cacc_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cacc_topic, qos_profile, std::bind(&CmdVelMuxPanel::caccCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", cacc_topic.c_str());

  teleop_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    teleop_topic, qos_profile,
    std::bind(&CmdVelMuxPanel::teleopCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", teleop_topic.c_str());

  nav2_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    nav2_topic, qos_profile, std::bind(&CmdVelMuxPanel::nav2Callback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", nav2_topic.c_str());
}

// Callback for CACC messages
void CmdVelMuxPanel::caccCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cacc_msg_ = *msg;
  cacc_msg_received_ = true;
  cacc_last_msg_time_ = std::chrono::steady_clock::now();
}

// Callback for Teleop messages
void CmdVelMuxPanel::teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_teleop_msg_ = *msg;
  teleop_msg_received_ = true;
  teleop_last_msg_time_ = std::chrono::steady_clock::now();
}

// Callback for Nav2 messages
void CmdVelMuxPanel::nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_nav2_msg_ = *msg;
  nav2_msg_received_ = true;
  nav2_last_msg_time_ = std::chrono::steady_clock::now();
}

// Timer callback to publish the selected command
void CmdVelMuxPanel::publishCommand()
{
  if (!cmd_vel_publisher_ || current_namespace_.empty()) {
    return;
  }

  // Create twist message based on *activated* source
  geometry_msgs::msg::Twist msg_to_publish = createZeroTwist();  // Default to zero
  bool message_available = false;
  SelectedSource source_to_publish = activated_source_;  // Use activated source

  switch (source_to_publish) {
    case SelectedSource::CACC:
      if (cacc_msg_received_ && isMessageRecent(cacc_last_msg_time_)) {
        msg_to_publish = latest_cacc_msg_;
        message_available = true;
      }
      break;

    case SelectedSource::TELEOP:
      if (teleop_msg_received_ && isMessageRecent(teleop_last_msg_time_)) {
        msg_to_publish = latest_teleop_msg_;
        message_available = true;
      }
      break;

    case SelectedSource::NAV2:
      if (nav2_msg_received_ && isMessageRecent(nav2_last_msg_time_)) {
        msg_to_publish = latest_nav2_msg_;
        message_available = true;
      }
      break;

    case SelectedSource::OFF:
    default:
      // Already set to zero twist
      message_available = true;  // OFF is always "available"
      break;
  }

  // Publish the message
  cmd_vel_publisher_->publish(msg_to_publish);

  // Log status occasionally (every ~5 seconds)
  static auto last_log_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (now - last_log_time > std::chrono::seconds(5)) {
    const char * source_name = "OFF";  // Default to OFF
    if (source_to_publish != SelectedSource::OFF) {
      switch (source_to_publish) {
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
          source_name = "UNKNOWN";
          break;  // Should not happen
      }
      RCLCPP_INFO(
        node_->get_logger(), "Publishing cmd_vel from ACTIVATED source: %s, valid message: %s",
        source_name, (message_available ? "yes" : "no (using zero)"));
    } else {
      RCLCPP_INFO(node_->get_logger(), "Publishing cmd_vel from ACTIVATED source: OFF");
    }
    last_log_time = now;
  }
}

// Helper to create a zero Twist message
geometry_msgs::msg::Twist CmdVelMuxPanel::createZeroTwist()
{
  geometry_msgs::msg::Twist zero_twist;
  zero_twist.linear.x = 0.0;
  zero_twist.linear.y = 0.0;
  zero_twist.linear.z = 0.0;
  zero_twist.angular.x = 0.0;
  zero_twist.angular.y = 0.0;
  zero_twist.angular.z = 0.0;
  return zero_twist;
}

// Load configuration - load namespace and selected source
void CmdVelMuxPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString ns;
  int staged_source_index = 0;     // Load staged source index
  int activated_source_index = 0;  // Load activated source index

  // Load namespace first, as changing it resets sources
  if (config.mapGetString("Namespace", &ns)) {
    // Temporarily block signals to prevent onNamespaceChanged from resetting loaded sources later
    namespace_input_->blockSignals(true);
    namespace_input_->setText(ns);
    namespace_input_->blockSignals(false);
    // Manually call the handler *after* loading other settings if namespace changed
    onNamespaceChanged(ns);  // This will reset sources, which is fine before loading them
  }

  // Load staged source index
  if (config.mapGetInt("StagedSourceIndex", &staged_source_index)) {
    if (staged_source_index >= 0 && staged_source_index < source_selector_->count()) {
      // Block signals to prevent onSourceSelected from potentially overriding loaded
      // activated_source
      source_selector_->blockSignals(true);
      source_selector_->setCurrentIndex(staged_source_index);
      source_selector_->blockSignals(false);
      // Manually update the internal state
      onSourceSelected(staged_source_index);
    }
  }
  // Load activated source index
  if (config.mapGetInt("ActivatedSourceIndex", &activated_source_index)) {
    if (
      activated_source_index >= 0 &&
      activated_source_index <= static_cast<int>(SelectedSource::NAV2)) {
      // Directly set the activated source state
      activated_source_ = static_cast<SelectedSource>(activated_source_index);
      RCLCPP_INFO(node_->get_logger(), "Loaded activated source: %d", activated_source_index);
    }
  }
}

// Save configuration - save namespace and selected source
void CmdVelMuxPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  config.mapSetValue("Namespace", namespace_input_->text());
  // Save the staged source index (current dropdown selection)
  config.mapSetValue("StagedSourceIndex", source_selector_->currentIndex());
  // Save the activated source index
  config.mapSetValue("ActivatedSourceIndex", static_cast<int>(activated_source_));
}

}  // namespace auna_rviz_plugins

// Export the plugin class.
#include <pluginlib/class_list_macros.hpp>
// Ensure proper namespace visibility
PLUGINLIB_EXPORT_CLASS(auna_rviz_plugins::CmdVelMuxPanel, rviz_common::Panel)