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

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <vector>

#include "laser_proc/laser_proc.hpp"

namespace laser_proc
{

sensor_msgs::msg::LaserScan LaserProc::getFirstScan(
  const sensor_msgs::msg::MultiEchoLaserScan & msg)
{
  sensor_msgs::msg::LaserScan out;
  fillLaserScan(msg, out);
  out.ranges.resize(msg.ranges.size());
  if (msg.ranges.size() == msg.intensities.size()) {
    out.intensities.resize(msg.intensities.size());
  }

  for (size_t i = 0; i < out.ranges.size(); i++) {
    size_t index = getFirstValue(msg.ranges[i], out.ranges[i]);
    if (out.intensities.size() > 0) {
      if (msg.intensities[i].echoes.size() > 0) {
        out.intensities[i] = msg.intensities[i].echoes[index];
      } else {
        out.intensities[i] = 0.0;
      }
    }
  }
  return out;
}

sensor_msgs::msg::LaserScan LaserProc::getLastScan(const sensor_msgs::msg::MultiEchoLaserScan & msg)
{
  sensor_msgs::msg::LaserScan out;
  fillLaserScan(msg, out);
  out.ranges.resize(msg.ranges.size());
  if (msg.ranges.size() == msg.intensities.size()) {
    out.intensities.resize(msg.intensities.size());
  }
  for (size_t i = 0; i < out.ranges.size(); i++) {
    size_t index = getLastValue(msg.ranges[i], out.ranges[i]);
    if (out.intensities.size() > 0) {
      if (msg.intensities[i].echoes.size() > 0) {
        out.intensities[i] = msg.intensities[i].echoes[index];
      } else {
        out.intensities[i] = 0.0;
      }
    }
  }
  return out;
}

sensor_msgs::msg::LaserScan LaserProc::getMostIntenseScan(
  const sensor_msgs::msg::MultiEchoLaserScan & msg)
{
  sensor_msgs::msg::LaserScan out;
  fillLaserScan(msg, out);
  if (msg.ranges.size() == msg.intensities.size()) {
    out.ranges.resize(msg.ranges.size());
    out.intensities.resize(msg.intensities.size());
  } else {
    std::stringstream ss;
    ss <<
      "getMostIntenseScan::Size of ranges does not equal size of intensities, cannot create scan.";
    fprintf(stderr, "%s\n", ss.str().c_str());
  }
  for (size_t i = 0; i < out.intensities.size(); i++) {
    getMostIntenseValue(msg.ranges[i], msg.intensities[i], out.ranges[i], out.intensities[i]);
  }
  return out;
}

void LaserProc::fillLaserScan(
  const sensor_msgs::msg::MultiEchoLaserScan & msg,
  sensor_msgs::msg::LaserScan & out)
{
  out.header = msg.header;
  out.angle_min = msg.angle_min;
  out.angle_max = msg.angle_max;
  out.angle_increment = msg.angle_increment;
  out.time_increment = msg.time_increment;
  out.scan_time = msg.scan_time;
  out.range_min = msg.range_min;
  out.range_max = msg.range_max;
}

///< @TODO I'm assuming all laserscanners/drivers output the ranges
// in order received (shortest to longest).
// If this is not the case, please make an issue.
size_t LaserProc::getFirstValue(const sensor_msgs::msg::LaserEcho & ranges, float & range)
{
  if (ranges.echoes.size() > 0) {
    size_t index = 0;
    range = ranges.echoes[index];
    return index;
  }

  range = std::numeric_limits<float>::quiet_NaN();
  return 0;  // Value doesn't matter
}

///< @TODO I'm assuming all laserscanners/drivers output the ranges
// in order received (shortest to longest).
// If this is not the case, please make an issue.
size_t LaserProc::getLastValue(const sensor_msgs::msg::LaserEcho & ranges, float & range)
{
  if (ranges.echoes.size() > 0) {
    size_t index = ranges.echoes.size() - 1;
    range = ranges.echoes[index];
    return index;
  }

  range = std::numeric_limits<float>::quiet_NaN();
  return 0;  // Value doesn't matter
}

void LaserProc::getMostIntenseValue(
  const sensor_msgs::msg::LaserEcho & ranges,
  const sensor_msgs::msg::LaserEcho & intensities, float & range,
  float & intensity)
{
  if (intensities.echoes.size() == 0) {
    range = std::numeric_limits<float>::quiet_NaN();
    intensity = 0.0;
    return;
  }

  std::vector<float>::const_iterator max_iter = std::max_element(
    intensities.echoes.begin(), intensities.echoes.end());
  size_t index = std::distance(intensities.echoes.begin(), max_iter);

  if (ranges.echoes.size() > 0) {
    range = ranges.echoes[index];
    intensity = *max_iter;
  } else {
    range = std::numeric_limits<float>::quiet_NaN();
    intensity = 0.0;
    return;
  }
}

}  // namespace laser_proc
