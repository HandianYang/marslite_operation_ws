# MARS-lite Operation ROS Workspace 

This ROS workspace provides robot operations for MARS-lite robots, such as navigation and robotic arm control.

## Prerequisites

### Hardware (PC)
- CUDA 12.1-capable GPU

### Software (Host machine)
- Ubuntu Host: 16.04 LTS ~ 24.04 LTS
- NVIDIA Driver: 525.60+
- NVIDIA Container Toolkit: latest
- Docker Engine: 20.10+

## Setup

### Docker setup

The environment for this workspace was built using Docker.

#### Enter Docker container

To create a Docker container named `marslite-operation` based on the latest version of Docker image `handianyang/marslite-operation:ros1-noetic`, and then enter this container:

```bash
source docker_run.sh
```

If the container with name `marslite-operation` already exists, it will be **reopened**.

Aditionally, you can find more details (installed package list, tag list, etc.) on [Docker Hub](https://hub.docker.com/repository/docker/handianyang/marslite-operation/general).


#### Remove Docker container

```bash
docker rm marslite-operation
```

### MARS-lite robot bringup

#### Bringup with Moveit! controller

```bash
# on MARS-lite
roslaunch mars_lite_bringup mars_lite_bringup.launch
```

#### Bringup with cartesian controller

```bash
# on MARS-lite
cd /home/kyyoung/cartesian_controller_ws
source devel/setup.bash
roslaunch controller_bringup motion_controller_bringup.launch 
```

### MARS-lite robot simulation setup

#### Bringup with Moveit! controller

```bash
# on Docker container
# under /home/developer/marslite_operation_ws 
source devel/setup.bash
source export_gazebo_simulation_model.sh
roslaunch marslite_simulation gazebo_supermarket.launch
```

#### Bringup with cartesian controller

**(Run once on first use)** Install `cartesian_controllers` ROS package from GitHub:
```bash
# on Docker container
# under /home/developer/marslite_operation_ws/src
apt update
git clone -b ros1 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Bringup with cartesian controller:
```bash
# on Docker container
# under /home/developer/marslite_operation_ws
source devel/setup.bash
source export_gazebo_simulation_model.sh
roslaunch marslite_simulation gazebo_supermarket.launch use_cartesian_controller:=true
```

## Instructions

### Robotic arm control

- Launch Moveit! control interface
  - Status
    - simulation: stable
    - real robot: **not tested**

```bash
roslaunch marslite_simulation moveit_planning_gz.launch
```

- Launch motion controller teleoperation module using Cartesian controller
  - Status
    - simulation: stable
    - real robot: stable

```bash
roslaunch marslite_control motion_controller_teleoperation.launch
```

- Launch `motion_controller_teleoperation.launch` with shared controller module 
  - Status
    - simulation: **not tested**
    - real robot: stable

```bash
roslaunch marslite_control motion_controller_teleoperation.launch
```

### Navigation

#### Launch SLAM using gmapping method

```bash
roslaunch marslite_navigation slam_gmapping.launch
```

#### Launch AMCL and navigation module using A* and DWA path-planning algorithm

```bash
roslaunch marslite_navigation navigation.launch
```

#### Launch mobile platform teleoperation using keyboard inputs

```bash
roslaunch marslite_navigation teleop_keyboard.launch
```

## Known Issues

- `marslite_control/motion_controller_teleoperation` calculates cylindrical pose wrong