# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2-based autonomous navigation system for the Unitree Go2 quadruped robot. The system implements SLAM, navigation, perception, and autonomous charging capabilities using a unified robot interface architecture that can potentially support multiple robot platforms.

## Core Architecture

The system follows a 4-layer architecture:
- **Application Layer**: Task management, map management, charging scheduler, system monitoring
- **Algorithm Layer**: SLAM (2D/3D), Nav2 navigation framework, point cloud processing, path planning, dynamic obstacle avoidance  
- **ROS2 Communication Layer**: Based on Cyclone DDS for direct communication with Go2
- **Robot Interface Layer**: Direct ROS2 topic interfaces with Go2 robot (motion control, state monitoring, sensor data, power management)

Key directories:
- `src/robot_interfaces/`: Unified robot interface layer with adapters for different robot platforms
- `src/navigation_core/`: Core navigation modules (SLAM, path planning, motion control, obstacle avoidance, localization)  
- `src/perception_core/`: Sensor data processing (LiDAR, camera, IMU, multi-sensor fusion)
- `src/charging_systems/`: Autonomous charging system with docking controllers
- `src/applications/`: High-level application modules (task manager, map manager, fleet manager)
- `src/third_party/go_ros2/`: Official Unitree Go2 ROS2 communication library

## Build and Development Commands

### Build System
```bash
# Build entire workspace
colcon build

# Build specific packages  
colcon build --packages-select <package_name>

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Environment Setup
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash  # or foxy

# Source workspace
source install/setup.bash

# For Go2 robot connection, configure network interface in setup script:
# export CYCLONEDDS_URI with correct NetworkInterface name (e.g., enp3s0)
# Set IP to 192.168.123.99 with mask 255.255.255.0
```

### Testing
```bash
# Run tests
colcon test

# View test results
colcon test-result --all
```

### Key ROS2 Topics for Go2 Robot
- `/api/sport/request` - Motion control commands (unitree_api::msg::Request)
- `/sportmodestate` - Robot motion state
- `/lowstate` - Low-level robot state  
- `/utlidar/cloud` - LiDAR point cloud data
- `/wirelesscontroller` - Wireless controller input

## Technical Details

### Communication Architecture
- Uses Cyclone DDS 0.10.2 for compatibility with Go2 robot
- Direct ROS2 topic communication without SDK wrapper layer
- Standard geometry_msgs, sensor_msgs, nav_msgs interfaces
- Robot-specific message conversion through adapters

### SLAM Implementation  
- Indoor 2D: Cartographer
- Outdoor 3D: FAST-LIO2  
- Sensor fusion: robot_localization EKF
- Multi-map management with automatic switching

### Navigation Framework
- Nav2 framework for path planning and control
- Costmap layers: static, obstacle, inflation
- DWB controller adapted for quadruped locomotion
- Dynamic obstacle avoidance using point cloud processing

### Autonomous Charging
- Battery monitoring and power management
- Multi-stage docking: global navigation → feature-based guidance → precise alignment
- Support for wireless charging systems
- Charging station detection and relative pose estimation

## Development Notes

- Robot detection and adapter selection is automatic based on network discovery
- Configuration-driven approach for supporting multiple robot platforms
- Capability matrix defines robot-specific parameters and limits
- Modular design allows independent development and testing of components
- Docker containerization support for consistent development environments

## Robot Capabilities (Go2 Specific)

- Max linear velocity: 1.5 m/s  
- Max angular velocity: 2.0 rad/s
- Locomotion modes: walk, trot, bound
- LiDAR: Livox Mid360 (40m range)
- Battery: 15000 mAh, wireless charging
- Physical: 845x405x320mm, 15kg

The system is designed to be extensible to other quadruped robots through the unified interface layer and capability matrix configuration.