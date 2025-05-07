# Marslite Operation ROS Workspace 

<!-- ## Prerequisites

### Hardware (PC)
- CUDA 12.1-capable GPU

### Software
- Ubuntu Host: 16.04 LTS ~ 24.04 LTS
- NVIDIA Driver: 525.60+
- NVIDIA Container Toolkit: latest
- Docker Engine: 20.10+

## Setup

### 1. Pull the specified Docker image from Docker Hub

```bash
docker pull handianyang/marslite_operation:cuda12.1.1-pytorch2.5.1-noetic-v1.0.0
```

**(OPTIONAL)** You can find more details (installed package list, tag list, etc.) on [Docker Hub](https://hub.docker.com/repository/docker/handianyang/marslite-operation/general).

### 2. Enter the Docker container


Under the root directory of the workspace:
```bash
source docker_run.sh
```

This command finds container named with `marslite` and brings you in. If the `marsltie` container does not exist, then it creates a new container.


### 3. (For `product_detection` package ONLY) Enter Python 3.10-based virtual environment

(INSIDE container) Activate the virtual environment:
```bash
# [alias] py
source ~/venv-3-10/bin/activate
```

**[NOTE] Because ROS Noetic packages are compiled against Python 3.8, ROS nodes cannot be compiled and executed under Python 3.10-based virtual environment. Therefore, the Python scripts in `product_detection` are treated as normal Python scripts rather than ROS nodes.** 

## Development

### (Recommended) Use `tmux` interface

After entering the container, simply type `tmux` to enter the tmux interface.
```bash
tmux
```

Some frequently used `tmux` shortcuts are listed here:
- Split the current pane with a horizontal line: `Ctrl+b` `"`
- Split the current pane with a vertical line: `Ctrl+b` `%`
- Switch to pane to the given direction:
    - `Ctrl+b` $\uparrow$
    - `Ctrl+b` $\downarrow$
    - `Ctrl+b` $\leftarrow$
    - `Ctrl+b` $\rightarrow$
- Resize current pane:
    - `Ctrl+b+`$\uparrow$ or `Ctrl+b` `Ctrl+`$\uparrow$
    - `Ctrl+b+`$\downarrow$ or `Ctrl+b` `Ctrl+`$\downarrow$
    - `Ctrl+b+`$\leftarrow$ or `Ctrl+b` `Ctrl+`$\leftarrow$
    - `Ctrl+b+`$\rightarrow$ or `Ctrl+b` `Ctrl+`$\rightarrow$
- Close current pane: `Ctrl+b` `x` (or type `exit` command)

- Enter copy mode (mouse/keyboard scrolling and copying allowed): `Ctrl+b` `[`
- Quit the copy mode: `q`

For more `tmux` commands, please refer to the [tmux command cheat sheet](https://tmuxcheatsheet.com/).

### Build the ROS workspace

(INSIDE container) Under the root of the workspace:
```bash
# [alias] cm
catkin_make
```

### Source the ROS workspace bash script

(INSIDE container) Under the root of the workspace:
```bash
# [alias] sd
source devel/setup.bash
```

###  Remove the Docker container

If you somehow **mess up with the existing container** (e.g. having trouble `apt-get update`), one option is to give up any changes you have made in this contaminated container.

(OUTSIDE container) Run the following command to remove the container named `marslite`:
```bash
docker rm marslite
```

[NOTE] Change the container name in the command accordingly if you've changed it in `docker_run.sh`.


## Instructions

[NOTE] Every command should be executed INSIDE the container.

### Launch ROS-sharp communication

**[Purpose]** To connect to Unity through WebSocket
```bash
roslaunch file_server ros_sharp_communication.launch
```

###  (Simulation ONLY)Launch Gazebo world

1. Launch a **Gazebo world of supermarket environment** with Marslite robot:
    ```bash
    roslaunch mars_lite_description gazebo_supermarket.launch
    ```

    (Optional) Launch with D435 camera
    ```bash
    roslaunch mars_lite_description gazebo_supermarket.launch realsense_enabled:=true
    ```

2. Spawn a Marslite robot in an **existing Gazebo world**:
    ```bash
    roslaunch mars_lite_description spawn_mars.launch
    ```
    **[NOTE] This SHALL NOT be launched with `gazebo_supermarket.launch`.**


### Navigation features

1. Launch **SLAM** using 'gmapping' method:
    ```bash
    roslaunch marslite_navigation slam_gmapping.launch
    ```

2. Launch **navigation** module using A* and DWA path-planning algorithm:
    ```bash
    roslaunch marslite_navigation navigation.launch
    ```

3. Directly drive the robot with keyboard inputs:
    ```bash
    roslaunch marslite_navigation teleop_keyboard.launch
    ```

4. Directly drive the robot with joystick inputs:
    ```bash
    roslaunch marslite_navigation teleop_joystick.launch
    ```


### Robotic arm control

1. Launch the Moveit! control interface
    ```bash
    roslaunch mars_lite_moveit_config mars_lite_moveit_planning_execution_gz.launch
    ``` -->