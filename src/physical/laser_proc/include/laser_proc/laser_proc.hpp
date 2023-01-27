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

#ifndef LASER_PROC__LASER_PROC_HPP_
#define LASER_PROC__LASER_PROC_HPP_

#include "sensor_msgs/msg/laser_echo.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"

#include "visibility_control.hpp"

namespace laser_proc
{
class LaserProc
{
public:
  LASER_PROC_PUBLIC
  static sensor_msgs::msg::LaserScan getFirstScan(
    const sensor_msgs::msg::MultiEchoLaserScan & msg);

  LASER_PROC_PUBLIC
  static sensor_msgs::msg::LaserScan getLastScan(
    const sensor_msgs::msg::MultiEchoLaserScan & msg);

  LASER_PROC_PUBLIC
  static sensor_msgs::msg::LaserScan getMostIntenseScan(
    const sensor_msgs::msg::MultiEchoLaserScan & msg);

private:
  static void fillLaserScan(
    const sensor_msgs::msg::MultiEchoLaserScan & msg, sensor_msgs::msg::LaserScan & out);

  static size_t getFirstValue(
    const sensor_msgs::msg::LaserEcho & ranges, float & range);

  static size_t getLastValue(
    const sensor_msgs::msg::LaserEcho & ranges, float & range);

  static void getMostIntenseValue(
    const sensor_msgs::msg::LaserEcho & ranges,
    const sensor_msgs::msg::LaserEcho & intensities,
    float & range,
    float & intensity);
};
}  // namespace laser_proc

#endif  // LASER_PROC__LASER_PROC_HPP_
