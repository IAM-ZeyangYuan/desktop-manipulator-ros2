# Desktop Manipulator — ROS2

A 4-DOF RRPR desktop manipulator with trajectory planning,
self-collision checking, and ROS2 integration using ros2_control.

![Demo](docs/images/rviz_demo.gif)

## Overview

This project implements a desktop manipulator designed for simple
pick-and-place tasks. The manipulator has 3 revolute joints and
1 prismatic joint with an L-shaped end effector.

The project includes:
- Forward and inverse kinematics
- Workspace analysis using 3D alpha shapes
- Jacobian-based sensitivity analysis
- Trajectory planning (cubic spline + parabolic blend)
- Self-collision checking along planned trajectories
- Full ROS2 integration with ros2_control and JointTrajectoryController

## System architecture

![Architecture](docs/images/architecture.png)

## Packages

**manipulator_description** — URDF/xacro robot model, launch files,
ros2_control hardware configuration, RViz config.

**manipulator_planning** — Trajectory interpolation, action client
that sends goals to JointTrajectoryController, forward kinematics,
RViz marker visualization for waypoints and trajectory path.

**manipulator_interfaces** — Custom service definition for
waypoint-based trajectory planning.

## Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy
- ros2_control, ros2_controllers

## Build

```bash
mkdir -p ~/manipulator_ws/src
cd ~/manipulator_ws/src
git clone https://github.com/IAM-ZeyangYuan/desktop-manipulator-ros2.git .
cd ~/manipulator_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Run

Launch the ros2_control pipeline:

```bash
ros2 launch manipulator_description ros2_control.launch.py
```

In a second terminal, execute the trajectory:

```bash
ros2 run manipulator_planning trajectory_action_client
```

## Results

### Trajectory planning
![Trajectory](docs/images/trajectory_plot.png)

### Workspace
![Workspace](docs/images/workspace.png)

### Self-collision check
![Collision](docs/images/self_collision.png)

## Technologies

ROS2 Jazzy, ros2_control, Python, NumPy, SciPy, PyVista, xacro