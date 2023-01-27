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

#ifndef LASER_PROC__LASER_PUBLISHER_HPP_
#define LASER_PROC__LASER_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"

#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"

#include "visibility_control.hpp"

namespace laser_proc
{

class LaserPublisher
{
public:
  LASER_PROC_PUBLIC
  explicit LaserPublisher(
    rclcpp::Node::SharedPtr & nh, uint32_t queue_size, bool publish_echoes = true);

  LASER_PROC_PUBLIC
  explicit LaserPublisher(
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface,
    uint32_t queue_size,
    bool publish_echoes = true);

  /*!
   * \brief Returns the number of subscribers that are currently connected to
   * this LaserPublisher.
   *
   * Returns sum of all child publishers.
   */
  LASER_PROC_PUBLIC
  size_t getNumSubscribers() const;

  /*!
   * \brief Returns the topics of this LaserPublisher.
   */
  LASER_PROC_PUBLIC
  std::vector<std::string> getTopics() const;

  /*!
   * \brief Publish a MultiEchoLaserScan on the topics associated with this LaserPublisher.
   */
  LASER_PROC_PUBLIC
  void publish(const sensor_msgs::msg::MultiEchoLaserScan & msg) const;

  /*!
   * \brief Publish a MultiEchoLaserScan on the topics associated with this LaserPublisher.
   */
  LASER_PROC_PUBLIC
  void publish(sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg) const;

  /*!
   * \brief Shutdown the advertisements associated with this Publisher.
   */
  LASER_PROC_PUBLIC
  void shutdown();

  LASER_PROC_PUBLIC
  operator void *() const;
  LASER_PROC_PUBLIC
  bool operator<(const LaserPublisher & rhs) const {return impl_ < rhs.impl_;}
  LASER_PROC_PUBLIC
  bool operator!=(const LaserPublisher & rhs) const {return impl_ != rhs.impl_;}
  LASER_PROC_PUBLIC
  bool operator==(const LaserPublisher & rhs) const {return impl_ == rhs.impl_;}

private:
  struct Impl;
  typedef std::shared_ptr<Impl> ImplPtr;
  typedef std::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;
};
}  // namespace laser_proc

#endif  // LASER_PROC__LASER_PUBLISHER_HPP_
