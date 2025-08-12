# `marslite_control` Package

## Tests

**[NOTE] Remember to launch controllers in `cartesian_controller_ws` first!**

### `motion_controller_teleoperation`

This test implements the Cartesian teleoperation of the robotic arm, driven by the pose of the VR grips. The grip's pose is passed and recognized as a ROS topic called `/unity/joy_pose/left` in `PoseStamped` type. 

```Shell
roslaunch marslite_control motion_controller_teleoperation.launch
```

### `keyboard_teleoperation`

This test implements the Cartesian teleoperation of the robotic arm, driven by keyboard signals. 

```Shell
roslaunch marslite_control keyboard_teleoperation.launch
```

### `motion_controller_shared_control`

```Shell
roslaunch marslite_control motion_controller_shared_control.launch
```

### `keyboard_shared_control`

```Shell
roslaunch marslite_control keyboard_shared_control.launch
```