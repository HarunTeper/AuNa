# auna_wallfollowing

LiDAR-based wall-following controller for AuNa. The node subscribes to a
`sensor_msgs/LaserScan` topic and publishes
`ackermann_msgs/AckermannDriveStamped` commands that keep the vehicle at a
desired distance from the wall on its left.

## Method

Two range measurements are taken at angles `angle_a` and `angle_b`. From the
pair, the node estimates the vehicle's heading relative to the wall (`alpha`)
and its current perpendicular distance, then projects that distance forward by
`lookahead_distance` to obtain the error used for control. Steering is a PID
function of that error; the commanded speed drops to `min_velocity` when the
error exceeds `error_threshold` and is `max_velocity` otherwise.

The integral and derivative terms are scaled by the measured elapsed time
between callbacks, so `ki` and `kd` keep their physical meaning (1/s and s)
independently of the LiDAR rate. Periods outside `[min_dt, max_dt]` are treated
as unreliable — a dropped scan, a paused simulation, a clock jump — and the
rate-dependent terms are held for that cycle.

## Topics

| Direction | Topic (default) | Type |
| --- | --- | --- |
| Subscribe | `scan` | `sensor_msgs/LaserScan` |
| Publish | `cmd_vel/wallfollowing` | `ackermann_msgs/AckermannDriveStamped` |

Both are configurable via the `lidarscan_topic` and `drive_topic` parameters.
The output topic is one of the input sources of the `auna_control` command
multiplexer; select `wallfollowing` in the RViz control panel to drive with it.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `kp` | 0.25 | Proportional gain |
| `ki` | 0.0 | Integral gain (1/s) |
| `kd` | 0.0 | Derivative gain (s) |
| `desired_distance` | 0.5 | Target distance to the wall (m) |
| `velocity` | 1.5 | Base velocity (m/s) |
| `min_velocity` | 1.0 | Velocity when the error exceeds `error_threshold` (m/s) |
| `max_velocity` | 2.0 | Velocity when the error is within `error_threshold` (m/s) |
| `error_threshold` | 1.0 | Error above which the vehicle slows down (m) |
| `max_steering_angle` | 0.4189 | Steering limit (rad, ~24°) |
| `angle_a` | π/4 | First scan angle used for the wall estimate (rad) |
| `angle_b` | π/2 | Second scan angle, perpendicular to the vehicle (rad) |
| `lookahead_distance` | 1.0 | Forward projection of the distance error (m) |
| `min_dt` | 0.001 | Shortest plausible control period (s) |
| `max_dt` | 0.5 | Longest plausible control period (s) |
| `max_integral` | 1.0 | Anti-windup clamp on the integral term |
| `lidarscan_topic` | `scan` | Input scan topic |
| `drive_topic` | `cmd_vel/wallfollowing` | Output drive topic |

Defaults are set in `auna_common/config/wallfollowing/wallfollowing.yaml`.

## Running

Within a scenario, the controller is started by the `wallfollowing` Compose
service. Standalone:

```bash
ros2 launch auna_wallfollowing wallfollowing.launch.py
```

## Tests

```bash
colcon test --packages-select auna_wallfollowing
colcon test-result --verbose
```

`test/test_wallfollowing_pid.cpp` verifies that the integral and derivative
terms are invariant to the callback rate, that an implausible period holds those
terms, and that the anti-windup clamp bounds the integral in both directions.

## Tuning note

`ki` and `kd` are interpreted as true integral and derivative gains. Gain sets
tuned before elapsed-time scaling was introduced need to be converted: divide
the old `kd` by the callback period and multiply the old `ki` by it. The shipped
defaults are zero for both, so the default configuration is unaffected.
