/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey
 */

#include <memory>
#include <string>
#include <vector>

#include "laser_proc/laser_proc.hpp"
#include "laser_proc/laser_publisher.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_echo.hpp"

namespace laser_proc
{

typedef sensor_msgs::msg::LaserScan (* PublishFunction)(
  const sensor_msgs::msg::MultiEchoLaserScan &
  msg);

struct LaserPublisher::Impl
{
  Impl()
  : unadvertised_(false)
  {}

  ~Impl()
  {
    shutdown();
  }

  bool isValid() const
  {
    return !unadvertised_;
  }

  void shutdown()
  {
    if (!unadvertised_) {
      unadvertised_ = true;
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::MultiEchoLaserScan>::SharedPtr echo_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> pubs_;
  std::vector<PublishFunction> functs_;
  bool unadvertised_;
};

///< @TODO Make a static class that creates these
LaserPublisher::LaserPublisher(
  rclcpp::Node::SharedPtr & nh, uint32_t queue_size, bool publish_echoes)
: LaserPublisher(nh->get_node_topics_interface(), queue_size, publish_echoes)
{}

LaserPublisher::LaserPublisher(
  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> nh,
  uint32_t queue_size,
  bool publish_echoes)
: impl_(new Impl)
{
  if (publish_echoes) {
    impl_->echo_pub_ =
      rclcpp::create_publisher<sensor_msgs::msg::MultiEchoLaserScan>(nh, "echoes", queue_size);
  }

  impl_->pubs_.push_back(
    rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(nh, "first", queue_size));
  impl_->functs_.push_back(laser_proc::LaserProc::getFirstScan);

  impl_->pubs_.push_back(
    rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(nh, "last", queue_size));
  impl_->functs_.push_back(laser_proc::LaserProc::getLastScan);

  impl_->pubs_.push_back(
    rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(nh, "most_intense", queue_size));
  impl_->functs_.push_back(laser_proc::LaserProc::getMostIntenseScan);
}

size_t LaserPublisher::getNumSubscribers() const
{
  size_t num = 0;

  if (impl_ && impl_->isValid()) {
    num += impl_->echo_pub_->get_subscription_count();
    for (const auto & pub : impl_->pubs_) {
      num += pub->get_subscription_count();
    }
  }

  return num;
}

std::vector<std::string> LaserPublisher::getTopics() const
{
  std::vector<std::string> topics;
  topics.push_back(impl_->echo_pub_->get_topic_name());
  if (impl_ && impl_->isValid()) {
    for (size_t i = 0; i < impl_->pubs_.size(); i++) {
      topics.push_back(impl_->pubs_[i]->get_topic_name());
    }
  }
  return topics;
}

void LaserPublisher::publish(const sensor_msgs::msg::MultiEchoLaserScan & msg) const
{
  if (!impl_ || !impl_->isValid()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("laser_publisher"),
      "Call to publish() on an invalid image_transport::LaserPublisher");
    return;
  }

  // Publish original MultiEchoLaserScan
  if (impl_->echo_pub_) {
    if (impl_->echo_pub_->get_subscription_count() > 0) {
      impl_->echo_pub_->publish(msg);
    }
  }

  // If needed, publish LaserScans
  for (size_t i = 0; i < impl_->pubs_.size(); i++) {
    if (impl_->pubs_[i]->get_subscription_count() > 0) {
      try {
        impl_->pubs_[i]->publish(impl_->functs_[i](msg));
      } catch (std::runtime_error & e) {
        auto err = std::string("Could not publish to topic ") + impl_->pubs_[i]->get_topic_name();
        auto logger = rclcpp::get_logger("laser_publisher");
        RCLCPP_ERROR(logger, "%s", err.c_str());
        RCLCPP_ERROR(logger, e.what());
      }
    }
  }
}

void LaserPublisher::publish(sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg) const
{
  if (!impl_ || !impl_->isValid()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("laser_publisher"),
      "Call to publish() on an invalid image_transport::LaserPublisher");
    return;
  }

  sensor_msgs::msg::MultiEchoLaserScan m = *msg;
  // Publish original MultiEchoLaserScan
  if (impl_->echo_pub_) {
    impl_->echo_pub_->publish(m);
  }

  // If needed, publish LaserScans
  for (size_t i = 0; i < impl_->pubs_.size(); i++) {
    if (impl_->pubs_[i]->get_subscription_count() > 0) {
      try {
        impl_->pubs_[i]->publish(impl_->functs_[i](m));
      } catch (std::runtime_error & e) {
        auto err = std::string("Could not publish to topic ") + impl_->pubs_[i]->get_topic_name();
        auto logger = rclcpp::get_logger("laser_publisher");
        RCLCPP_ERROR(logger, "%s", err.c_str());
        RCLCPP_ERROR(logger, e.what());
      }
    }
  }
}

void LaserPublisher::shutdown()
{
  if (impl_) {
    impl_->shutdown();
    impl_.reset();
  }
}

LaserPublisher::operator void *() const
{
  return (impl_ && impl_->isValid()) ? reinterpret_cast<void *>(1) : reinterpret_cast<void *>(0);
}

}  // namespace laser_proc
