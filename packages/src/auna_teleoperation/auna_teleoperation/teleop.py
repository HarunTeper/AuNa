#!/usr/bin/env python3
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
        self.declare_parameter("namespaces", [""])
        self.declare_parameter("default_namespace", "")

        self.LINEAR_MAX = self.get_parameter("linear_max").value
        self.ANGULAR_MAX = self.get_parameter("angular_max").value
        self.linear_speed_multiplier = self.get_parameter(
            "linear_speed_multiplier").value
        self.angular_speed_multiplier = self.get_parameter(
            "angular_speed_multiplier").value
        self.namespaces = self.get_parameter("namespaces").value
        self.default_namespace = self.get_parameter("default_namespace").value

        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self.publishers_ = {}
        self.twist_stamped_enabled = self.get_parameter(
            "twist_stamped_enabled").value

        for namespace in self.namespaces:
            if self.twist_stamped_enabled:
                self.publishers_[namespace] = self.create_publisher(
                    TwistStamped, namespace+"/cmd_vel/teleop", qos_profile_system_default)
            else:
                self.publishers_[namespace] = self.create_publisher(
                    Twist, namespace+"/cmd_vel/teleop", qos_profile_system_default)

        if self.default_namespace in self.publishers_:
            self.publisher_ = self.publishers_[self.default_namespace]
            self.active_namespace = self.default_namespace
        else:
            self.publisher_ = next(iter(self.publishers_.values()))
            self.active_namespace = next(iter(self.publishers_.keys()))

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
                    f"Trying to set a linear speed {linear} outside of allowed range of [{-self.LINEAR_MAX}, {self.LINEAR_MAX}]"
                )
        if angular is not None:
            if abs(angular) <= self.ANGULAR_MAX:
                self.angular = angular
            else:
                self.get_logger().error(
                    f"Trying to set a angular speed {angular} outside of allowed range of [{-self.ANGULAR_MAX}, {self.ANGULAR_MAX}]"
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

    def switch_namespace(self, direction):
        current_index = self.namespaces.index(self.active_namespace)
        if direction == "next":
            new_index = (current_index + 1) % len(self.namespaces)
        else:
            new_index = (current_index - 1) % len(self.namespaces)
        self.active_namespace = self.namespaces[new_index]
        self.publisher_ = self.publishers_[self.active_namespace]
        self.get_logger().info(
            f"Switched to namespace: {self.active_namespace}")
