#!/usr/bin/env python3
import os
import signal

import rclpy
from pynput.keyboard import Key, Listener

from auna_teleoperation.teleop import Teleop


class HoldKeyTeleop(Teleop):
    def __init__(self):
        super().__init__()
        self.SPEED_STEP = 0.1
        self.key_listener = Listener(
            on_press=self.update_twist,
            on_release=self.on_release,
            suppress=True  # Add this line to suppress keyboard echo
        )
        self.key_listener.start()
        self.keys_bindings = {
            "w": (self.LINEAR_MAX, 0.0),
            "s": (-self.LINEAR_MAX, 0.0),
            "a": (0.0, self.ANGULAR_MAX),
            "d": (0.0, -self.ANGULAR_MAX),
        }
        self.special_keys_bindings = {
            Key.up: (self.LINEAR_MAX, 0.0),
            Key.down: (-self.LINEAR_MAX, 0.0),
            Key.left: (0.0, self.ANGULAR_MAX),
            Key.right: (0.0, -self.ANGULAR_MAX),
        }
        self.pressed_keys = set()  # Track currently pressed keys
        self.get_logger().info(
            f"""
This node takes keypresses from the keyboard and publishes them 
as Twist messages. This is the holding mode; your keypress will
set the maximum configured speeds, at release all speeds are reset

WARNING: This node will take commands even if your terminal is not in focus!

Controls:

WASD or Arrows to move
i to increase forward speed (0.1 step)
k to decrease forward speed (0.1 step)
u to increase turning speed (0.1 step)
j to decrease turning speed (0.1 step)
Any other key to stop
CTRL-C or q to quit

Configuration:

Max Linear Speed: +/-{self.LINEAR_MAX} m/s
Max Angular Speed: +/-{self.ANGULAR_MAX} rad/s
"""
        )

    def __del__(self):
        # Add destructor to clean up the keyboard listener
        if hasattr(self, 'key_listener'):
            self.key_listener.stop()

    def compute_current_twist(self):
        linear = 0.0
        angular = 0.0

        # Check if we're moving backwards
        moving_backwards = 's' in self.pressed_keys or Key.down in self.pressed_keys

        # Check normal keys
        if 'w' in self.pressed_keys:
            linear = self.LINEAR_MAX * self.linear_speed_multiplier
        elif 's' in self.pressed_keys:
            linear = -self.LINEAR_MAX * self.linear_speed_multiplier
        if 'a' in self.pressed_keys:
            angular = self.ANGULAR_MAX * self.angular_speed_multiplier
            if moving_backwards:
                angular = -angular
        elif 'd' in self.pressed_keys:
            angular = -self.ANGULAR_MAX * self.angular_speed_multiplier
            if moving_backwards:
                angular = -angular

        # Check special keys
        if Key.up in self.pressed_keys:
            linear = self.LINEAR_MAX * self.linear_speed_multiplier
        elif Key.down in self.pressed_keys:
            linear = -self.LINEAR_MAX * self.linear_speed_multiplier
        if Key.left in self.pressed_keys:
            angular = self.ANGULAR_MAX * self.angular_speed_multiplier
            if moving_backwards:
                angular = -angular
        elif Key.right in self.pressed_keys:
            angular = -self.ANGULAR_MAX * self.angular_speed_multiplier
            if moving_backwards:
                angular = -angular

        return linear, angular

    def on_release(self, key):
        if self._is_special_key(key):
            if key in self.special_keys_bindings:
                self.pressed_keys.discard(key)
        else:
            key = key.char
            if key in self.keys_bindings:
                self.pressed_keys.discard(key)

        # Update twist based on remaining pressed keys
        linear, angular = self.compute_current_twist()
        self.write_twist(linear, angular)

    def update_twist(self, key):
        if self._is_special_key(key):
            if key in self.special_keys_bindings:
                self.pressed_keys.add(key)
                linear, angular = self.compute_current_twist()
                self.write_twist(linear, angular)
            else:
                self.write_twist(0.0, 0.0)
        else:
            if key.char == "q":
                os.kill(os.getpid(), signal.SIGINT)
            elif key.char == "i":
                self.linear_speed_multiplier = min(
                    1.0, self.linear_speed_multiplier + self.SPEED_STEP)
                self.get_logger().info(
                    f"Linear speed multiplier: {self.linear_speed_multiplier:.1f}")
                linear, angular = self.compute_current_twist()
                self.write_twist(linear, angular)
            elif key.char == "k":
                self.linear_speed_multiplier = max(
                    0.1, self.linear_speed_multiplier - self.SPEED_STEP)
                self.get_logger().info(
                    f"Linear speed multiplier: {self.linear_speed_multiplier:.1f}")
                linear, angular = self.compute_current_twist()
                self.write_twist(linear, angular)
            elif key.char == "u":
                self.angular_speed_multiplier = min(
                    1.0, self.angular_speed_multiplier + self.SPEED_STEP)
                self.get_logger().info(
                    f"Angular speed multiplier: {self.angular_speed_multiplier:.1f}")
                linear, angular = self.compute_current_twist()
                self.write_twist(linear, angular)
            elif key.char == "j":
                self.angular_speed_multiplier = max(
                    0.1, self.angular_speed_multiplier - self.SPEED_STEP)
                self.get_logger().info(
                    f"Angular speed multiplier: {self.angular_speed_multiplier:.1f}")
                linear, angular = self.compute_current_twist()
                self.write_twist(linear, angular)
            elif key.char in self.keys_bindings:
                self.pressed_keys.add(key.char)
                linear, angular = self.compute_current_twist()
                self.write_twist(linear, angular)
            else:
                self.write_twist(0.0, 0.0)

    def _is_special_key(self, key):
        try:
            key.char
            return False
        except AttributeError:
            return True


def main():
    try:
        rclpy.init()
        node = HoldKeyTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up the node properly
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
