# Wall Follow Node

This repository contains a ROS 2 node called `wallfollowing` which enables a robot to follow a wall using LIDAR sensor data and Ackermann steering control. The node subscribes to `/scan` (LIDAR scan data) and publishes driving commands to `/drive`.

---

## 📦 Features

- Uses PID control to maintain a desired distance from the wall (left side).
- Computes the angle between two LIDAR readings to estimate the vehicle's lateral position.

---

## 🛠 Dependencies

Ensure you have the following installed:

- ROS 2 (Foxy, Galactic, Humble, or later)
- `ackermann_msgs`
- `sensor_msgs`
- `nav_msgs`

## Staring the node

```bash
ros2 run wallfollowing wallfollowing
```
