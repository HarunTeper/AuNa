#!/usr/bin/env python3
import os
import signal
import threading

import rclpy
from pynput.keyboard import Key, Listener

from auna_teleoperation.teleop import Teleop


class HoldKeyTeleop(Teleop):
    def __init__(self):
        super().__init__()
        self.SPEED_STEP = 0.1
        self.lock = threading.Lock()  # Add a lock for thread safety
        self.key_listener = Listener(
            on_press=self.update_twist,
            on_release=self.on_release,
            daemon=True
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
        self.running = True  # Flag to indicate if node is running
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

    def close(self):
        """Properly clean up resources"""
        self.running = False
        if hasattr(self, 'key_listener'):
            if self.key_listener.is_alive():
                self.key_listener.stop()
                self.key_listener.join(timeout=1)  # Wait for thread to finish
        # Make sure the robot stops when shutting down
        self.write_twist(0.0, 0.0)

    def compute_current_twist(self):
        with self.lock:
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
        try:
            with self.lock:
                if self._is_special_key(key):
                    if key in self.special_keys_bindings:
                        self.pressed_keys.discard(key)
                else:
                    try:
                        key_char = key.char
                        if key_char in self.keys_bindings:
                            self.pressed_keys.discard(key_char)
                    except AttributeError:
                        pass  # Key doesn't have a char attribute

            # Update twist based on remaining pressed keys
            linear, angular = self.compute_current_twist()
            self.write_twist(linear, angular)
        except Exception as e:
            self.get_logger().error(f"Error in on_release: {str(e)}")
            self.write_twist(0.0, 0.0)  # Safety stop on error

    def update_twist(self, key):
        try:
            if not self.running:
                return

            if self._is_special_key(key):
                with self.lock:
                    if key in self.special_keys_bindings:
                        self.pressed_keys.add(key)
                linear, angular = self.compute_current_twist()
                self.write_twist(linear, angular)
            else:
                try:
                    key_char = key.char
                    if key_char == "q":
                        os.kill(os.getpid(), signal.SIGINT)
                    elif key_char == "i":
                        with self.lock:
                            self.linear_speed_multiplier = min(
                                1.0, self.linear_speed_multiplier + self.SPEED_STEP)
                        self.get_logger().info(
                            f"Linear speed multiplier: {self.linear_speed_multiplier:.1f}")
                        linear, angular = self.compute_current_twist()
                        self.write_twist(linear, angular)
                    elif key_char == "k":
                        with self.lock:
                            self.linear_speed_multiplier = max(
                                0.1, self.linear_speed_multiplier - self.SPEED_STEP)
                        self.get_logger().info(
                            f"Linear speed multiplier: {self.linear_speed_multiplier:.1f}")
                        linear, angular = self.compute_current_twist()
                        self.write_twist(linear, angular)
                    elif key_char == "u":
                        with self.lock:
                            self.angular_speed_multiplier = min(
                                1.0, self.angular_speed_multiplier + self.SPEED_STEP)
                        self.get_logger().info(
                            f"Angular speed multiplier: {self.angular_speed_multiplier:.1f}")
                        linear, angular = self.compute_current_twist()
                        self.write_twist(linear, angular)
                    elif key_char == "j":
                        with self.lock:
                            self.angular_speed_multiplier = max(
                                0.1, self.angular_speed_multiplier - self.SPEED_STEP)
                        self.get_logger().info(
                            f"Angular speed multiplier: {self.angular_speed_multiplier:.1f}")
                        linear, angular = self.compute_current_twist()
                        self.write_twist(linear, angular)
                    elif key_char in self.keys_bindings:
                        with self.lock:
                            self.pressed_keys.add(key_char)
                        linear, angular = self.compute_current_twist()
                        self.write_twist(linear, angular)
                except (AttributeError, TypeError):
                    pass  # Handle case where key.char doesn't exist
        except Exception as e:
            self.get_logger().error(f"Error in update_twist: {str(e)}")
            self.write_twist(0.0, 0.0)  # Safety stop on error

    def _is_special_key(self, key):
        try:
            key.char
            return False
        except (AttributeError, TypeError):
            return True


def main():
    rclpy.init()
    node = None
    try:
        node = HoldKeyTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Exception in main: {str(e)}")
    finally:
        # Clean up the node properly
        if node:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
