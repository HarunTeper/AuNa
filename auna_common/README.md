# AuNa Common Data

This directory contains centralized configuration files, maps, waypoints, worlds, and visualization configurations for the AuNa autonomous vehicle framework. By centralizing these resources, we avoid rebuilding ROS2 packages when only parameters or data files change.

## Directory Structure

```
auna_common/
├── config/                      # Configuration files
│   ├── cacc/                   # CACC controller parameters
│   ├── control/                # Control multiplexer parameters
│   ├── ekf/                    # Extended Kalman Filter parameters
│   ├── f110/                   # F1/10 platform-specific configs
│   ├── nav2/                   # Navigation2 stack parameters
│   ├── teleoperation/          # Teleoperation parameters
│   ├── wallfollowing/          # Wall following algorithm parameters
│   └── world_params/           # World-specific spawn and configuration
├── maps/                        # Map files for navigation
│   ├── arena/
│   └── racetrack_decorated/
├── waypoints/                   # Waypoint files for path following
│   ├── arena/
│   └── racetrack_decorated/
├── worlds/                      # Gazebo world files
└── rviz/                       # RViz visualization configurations
```

## Usage in Docker

This directory is mounted as a read-only volume in Docker containers:

```yaml
volumes:
  - ./auna_common:/home/ubuntu/workspace/auna_common:ro
```

## Configuration Files

### CACC (Cooperative Adaptive Cruise Control)
- `config/cacc/cacc_controller.yaml` - CACC controller parameters including gains, distances, and velocities

### Control
- `config/control/params.yaml` - Control multiplexer node parameters
- `config/control/topics.yaml` - Input/output topic mappings for command multiplexing

### EKF (Extended Kalman Filter)
- `config/ekf/ekf_global.yaml` - Global EKF localization parameters
- `config/ekf/ekf_local.yaml` - Local EKF localization parameters

### F1/10 Platform
- `config/f110/lidar_params.yaml` - LIDAR sensor configuration
- `config/f110/ukf.yaml` - Unscented Kalman Filter parameters
- `config/f110/teleop_joy.yaml` - Joystick teleoperation configuration
- `config/f110/vesc.config.yaml` - VESC motor controller configuration

### Navigation2
- `config/nav2/nav2_params.yaml` - Complete Navigation2 stack parameters

### Teleoperation
- `config/teleoperation/key_teleop.yaml` - Keyboard teleoperation parameters

### Wall Following
- `config/wallfollowing/wallfollowing.yaml` - Wall following algorithm parameters

### World Parameters
- `config/world_params/*.yaml` - Spawn positions and world-specific configurations

## Maps

Map files include:
- `map.yaml` - Map metadata (resolution, origin, etc.)
- `map.pgm` - Occupancy grid image

Available maps:
- `arena/` - Arena track map
- `racetrack_decorated/` - Decorated racetrack map

## Waypoints

Waypoint files are organized by track/scenario:
- YAML format waypoints for Nav2
- CSV format waypoints for custom controllers
- Different waypoint sets for various scenarios

## Worlds

Gazebo world files (`.world` format):
- `arena.world` - Arena environment
- `racetrack_decorated.world` - Decorated racetrack environment

## RViz Configurations

Pre-configured RViz setups for different use cases:
- `config_arena.rviz` - Arena visualization
- `config_mapping.rviz` - Mapping visualization
- `config_navigation_namespace.rviz` - Multi-robot navigation
- `control_panel.rviz` - Control panel interface
- `empty_config.rviz` - Minimal configuration

## Environment Variables

Several environment variables control which configurations are loaded:

- `MAP_NAME` - Selects which map to use (default: `racetrack_decorated`)
- `WORLD_NAME` - Selects which world to load (default: `racetrack_decorated`)
- `ROBOT_INDEX` - Robot number for multi-robot scenarios

## Accessing in Launch Files

Launch files should reference these files using absolute paths:

```python
# Example for accessing config files
config_file = '/home/ubuntu/workspace/auna_common/config/cacc/cacc_controller.yaml'

# Example for accessing maps
map_file = f'/home/ubuntu/workspace/auna_common/maps/{map_name}/map.yaml'

# Example for accessing waypoints
waypoint_file = f'/home/ubuntu/workspace/auna_common/waypoints/{map_name}/nav2_waypoints.yaml'
```

## Benefits

1. **No Rebuild Required** - Changing parameters doesn't require rebuilding packages
2. **Centralized Management** - All configurations in one place
3. **Version Control** - Easy to track parameter changes
4. **Multi-Robot Support** - Shared configurations across robot instances
5. **Environment-Specific** - Easy switching between simulation and real hardware configs
6. **Docker Efficiency** - Mount separately from source code

## Maintenance

When adding new configurations:
1. Place them in the appropriate subdirectory
2. Update this README
3. Update launch files to reference the new location
4. Ensure files are accessible with read-only mount
