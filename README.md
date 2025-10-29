# Franka Panda ROS2 with Robotiq Gripper Integration

This repository provides a complete ROS2 workspace for controlling a Franka Emika Panda robot arm with a Robotiq gripper. It integrates two main components into a unified workspace for seamless robotic manipulation.

## Overview

This workspace combines:

- **Franka Arm ROS2**: A comprehensive ROS2 port of `franka_ros` for Franka Emika Panda (FER) robots
- **ROS2 Robotiq Gripper**: ROS2 driver, controller, and description packages for Robotiq grippers

Together, these packages enable full control of a Panda arm equipped with a Robotiq gripper, supporting both individual component control and integrated manipulation tasks.

## Repository Structure

```
panda_ros2_humbe_robotiq/
├── franka_arm_ros2/              # Franka Panda arm ROS2 packages
│   ├── franka_bringup/           # Launch files and configurations
│   ├── franka_control2/          # Main control node
│   ├── franka_description/       # URDF/Xacro robot descriptions
│   ├── franka_example_controllers/ # Example controller implementations
│   ├── franka_gripper/           # Franka gripper integration
│   ├── franka_hardware/          # Hardware interface
│   ├── franka_moveit_config/     # MoveIt2 configuration
│   ├── franka_msgs/              # ROS2 message definitions
│   ├── franka_robot_state_broadcaster/ # Robot state publisher
│   └── franka_semantic_components/ # Semantic component interfaces
│
└── ros2_robotiq_gripper/         # Robotiq gripper ROS2 packages
    ├── robotiq_controllers/      # Gripper controllers
    ├── robotiq_description/      # Gripper URDF/meshes
    ├── robotiq_driver/           # Hardware driver
    └── robotiq_hardware_tests/   # Testing utilities
```

## Features

### Franka Arm Capabilities

**Single Arm:**
- FrankaState broadcaster for real-time robot status
- Multiple control interfaces (torque, position, velocity, Cartesian)
- Example controllers for all control modes
- Runtime controller switching via `rqt_controller_manager`
- Error recovery service via `~/service_server/error_recovery`
- Runtime parameter configuration services
- MoveIt2 integration for motion planning

**Multi-Arm:**
- Dual arm initialization and control
- Synchronized joint state broadcasting
- FrankaState broadcasting for both arms
- Swappable controllers
- Error recovery and parameter setters
- Dual joint impedance & velocity example controllers

### Robotiq Gripper Capabilities

- Support for multiple Robotiq gripper models (initially 2F-85)
- ROS2 control integration
- Hardware driver with serial communication
- Example controllers and configurations
- Compatible with ROS2 Humble, Iron, and Rolling

## Prerequisites

- **Operating System**: Ubuntu 22.04 (tested)
- **ROS2 Distribution**: Humble (tested)
- **Franka FCI Version**: 4.0.4, 4.2.1, or 4.2.2 (tested)
- **Libfranka**: 0.8.0 or 0.9.2

## Installation

### 1. Install Libfranka

Build libfranka from source following the [official instructions](https://frankaemika.github.io/docs/installation_linux.html).

Choose the appropriate version for your FCI:
- For direct installation of libfranka 0.9.2, use the [LCAS release](https://github.com/LCAS/libfranka/tree/lcas_0.9.2)

```bash
# Example installation
cd ~/Downloads
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka
git checkout 0.9.2  # or 0.8.0 depending on your FCI version
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

### 2. Set Up ROS2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone <your-repository-url> panda_ros2_humbe_robotiq

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

### 3. Configure Environment

Add libfranka to your library path:

```bash
echo 'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/path/to/libfranka/build"' >> ~/.bashrc
source ~/.bashrc
```

### 4. Optional: FLIR Camera Driver (for vision-based tasks)

For panda_vision setup, install the FLIR Blackfly_s camera driver:
- [FLIR Camera Driver installation](https://github.com/LCAS/flir_camera_driver)

## Quick Start

### Launch Franka Panda with MoveIt2

```bash
ros2 launch franka_moveit_config moveit_real_arm_platform.launch.py \
    robot_ip:=<robot-ip> \
    load_gripper:=True
```

Example:
```bash
ros2 launch franka_moveit_config moveit_real_arm_platform.launch.py \
    robot_ip:=172.16.0.2
```

### Launch with Camera Integration

```bash
ros2 launch franka_moveit_config moveit_real_arm_platform.launch.py \
    robot_ip:=172.16.0.2 \
    camera_type:=blackfly_s \
    serial:="'22141921'" \
    load_camera:=True
```

### Launch Dual Arm Setup

```bash
ros2 launch franka_bringup dual_franka.launch.py \
    robot_ip_1:=172.16.0.2 \
    robot_ip_2:=172.16.0.3
```

### Launch Example Controllers

```bash
# Joint impedance controller
ros2 launch franka_bringup joint_impedance_example_controller.launch.py robot_ip:=172.16.0.2

# Joint velocity controller
ros2 launch franka_bringup joint_velocity_example_controller.launch.py robot_ip:=172.16.0.2

# Gravity compensation
ros2 launch franka_bringup gravity_compensation_example_controller.launch.py robot_ip:=172.16.0.2
```

### Error Recovery

If the robot encounters an error during operation:

```bash
ros2 service call /service_server/error_recovery std_srvs/srv/Trigger
```

## Usage with Robotiq Gripper

### Basic Gripper Control

```bash
# Launch Robotiq gripper driver
ros2 launch robotiq_driver robotiq_gripper.launch.py

# Control gripper position (0.0 = open, 1.0 = closed)
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"
```

### Integrated Arm and Gripper Control

For combined manipulation tasks, you can use MoveIt2 with the gripper configured as an end-effector. Refer to the MoveIt2 configuration in `franka_moveit_config/` for details.

## Advanced Usage

### MoveIt2 Commander for Plant Scanning

For automated plant scanning and manipulation:
- [MoveIt2 Commander Recorder](https://github.com/LCAS/moveit2_commander_recorder)
- [Viewpoint Generator](https://github.com/LCAS/viewpoint_generator)

### Multi-Arm with MuJoCo Simulation

For an upgraded version including MuJoCo simulator support:
- [Multi Franka Arm ROS2](https://github.com/yilmazabdurrah/multi_franka_arm_ros2)

### Switching Controllers at Runtime

```bash
# List available controllers
ros2 control list_controllers

# Switch controllers
ros2 control switch_controllers --stop-controllers controller1 --start-controllers controller2
```

## Known Issues

- **Joint position controller**: May cause unstable motor behavior. Use torque or velocity controllers instead.
- **Joint limits**: Not fully enforced in `ros2_control`; must be implemented at the controller level.

## Troubleshooting

### Robot Won't Start

1. Ensure the robot is not in error state before launching
2. Check that the FCI version matches your libfranka version
3. Verify network connectivity: `ping <robot-ip>`
4. Check FCI interface is unlocked in the Franka Desk

### Controller Crashes

1. Use the error recovery service (see above)
2. Check real-time kernel configuration
3. Verify joint limits are appropriate for your workspace

### Build Errors

1. Ensure libfranka is correctly installed: `ldconfig -p | grep libfranka`
2. Verify ROS2 Humble is sourced: `echo $ROS_DISTRO`
3. Check all dependencies are installed: `rosdep check --from-paths src --ignore-src`

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request with a clear description of changes

## Credits and Attribution

### Franka Arm ROS2

This package is based on:
- Original fork: [mcbed's franka_ros2 for Humble](https://github.com/mcbed/franka_ros2/tree/humble)
- Addresses the gap left by Franka Emika's discontinued support for Panda (FER) robots in favor of FR3

### ROS2 Robotiq Gripper

This package is developed and maintained by:
- [PickNik Robotics](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
- Community contributors (not officially sponsored by Robotiq)

## License

### Franka Arm ROS2 Packages

All packages in `franka_arm_ros2/` are licensed under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0.html), following the licensing of `franka_ros2` and `panda_ros2`.

### ROS2 Robotiq Gripper Packages

Refer to individual package licenses in `ros2_robotiq_gripper/`.

## Support and Resources

- **Franka Control Interface Documentation**: [FCI Docs](https://frankaemika.github.io/docs)
- **ROS2 Control Documentation**: [ros2_control](https://control.ros.org)
- **MoveIt2 Documentation**: [MoveIt2](https://moveit.ros.org)

## Roadmap

Future improvements planned:
- Enhanced impedance controllers with proper subscribers
- Extensive tutorials and examples
- Better multi-arm MoveIt2 integration
- Additional Robotiq gripper model support
- Improved documentation and CI/CD

---

**Note**: This repository was created for rapid deployment and integration. The original upload contained a zip file structure which has been reorganized for proper version control and collaborative development.
