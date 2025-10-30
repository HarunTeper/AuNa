#!/usr/bin/env python3

# Copyright 2025 Harun Teper
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import atexit
import sys
from abc import ABC, abstractmethod

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class Teleop(Node, ABC):
    def __init__(self):
        atexit.register(self._emergency_stop)
        Node.__init__(self, "keyboard_teleop")

        self.declare_parameter("twist_stamped_enabled", False)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("linear_max", 1.0)
        self.declare_parameter("angular_max", 1.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("linear_speed_multiplier", 1.0)
        self.declare_parameter("angular_speed_multiplier", 1.0)
        self.declare_parameter("base_namespace", "robot")

        self.LINEAR_MAX = self.get_parameter("linear_max").value
        self.ANGULAR_MAX = self.get_parameter("angular_max").value
        self.linear_speed_multiplier = self.get_parameter(
            "linear_speed_multiplier").value
        self.angular_speed_multiplier = self.get_parameter(
            "angular_speed_multiplier").value
        self.base_namespace = self.get_parameter("base_namespace").value

        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self.publishers_ = {}
        self.twist_stamped_enabled = self.get_parameter(
            "twist_stamped_enabled").value

        self.namespace_index = 0
        self.active_namespace = self.base_namespace + str(self.namespace_index)
        self.publisher_ = self._create_publisher(self.active_namespace)

        if self.twist_stamped_enabled:
            self._make_twist = self._make_twist_stamped
        else:
            self._make_twist = self._make_twist_unstamped

        rate = 1 / self.get_parameter("publish_rate").value
        self.create_timer(rate, self._publish)
        self.linear = 0.0
        self.angular = 0.0

    @abstractmethod
    def update_twist(self, *args):
        pass

    def write_twist(self, linear=None, angular=None):
        if linear is not None:
            if abs(linear) <= self.LINEAR_MAX:
                self.linear = linear
            else:
                self.get_logger().error(
                    f"Trying to set a linear speed {linear} "
                    f"outside of allowed range of [{-self.LINEAR_MAX}, {self.LINEAR_MAX}]"
                )
        if angular is not None:
            if abs(angular) <= self.ANGULAR_MAX:
                self.angular = angular
            else:
                self.get_logger().error(
                    f"Trying to set a angular speed {angular} "
                    f"outside of allowed range of [{-self.ANGULAR_MAX}, {self.ANGULAR_MAX}]"
                )
        self._update_screen()

    def _make_twist_unstamped(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _make_twist_stamped(self, linear, angular):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self._robot_base_frame
        twist_stamped.twist = self._make_twist_unstamped(linear, angular)
        return twist_stamped

    def _publish(self):
        twist = self._make_twist(self.linear, self.angular)
        self.publisher_.publish(twist)

    def _update_screen(self):
        sys.stdout.write(
            f"Namespace: {self.active_namespace}, "
            f"Linear: {self.linear:.2f} (x{self.linear_speed_multiplier:.1f}), "
            f"Angular: {self.angular:.2f} (x{self.angular_speed_multiplier:.1f})\r")

    def _emergency_stop(self):
        for publisher in self.publishers_.values():
            publisher.publish(self._make_twist(0.0, 0.0))

    def _create_publisher(self, namespace):
        if namespace in self.publishers_:
            return self.publishers_[namespace]

        topic = f"/{namespace}/cmd_vel/teleop"
        if self.twist_stamped_enabled:
            publisher = self.create_publisher(
                TwistStamped, topic, qos_profile_system_default)
        else:
            publisher = self.create_publisher(
                Twist, topic, qos_profile_system_default)
        self.publishers_[namespace] = publisher
        return publisher

    def switch_namespace(self, direction):
        if direction == "next":
            self.namespace_index += 1
        else:
            self.namespace_index = max(0, self.namespace_index - 1)

        self.active_namespace = self.base_namespace + str(self.namespace_index)
        self.publisher_ = self._create_publisher(self.active_namespace)
        self.get_logger().info(
            f"Switched to namespace: {self.active_namespace}")
