# `marslite_control` Package

## Method

### Intent inference

#### Bayesian inference with Markov transition model

#### Finite state machine

Definition of `GripperMotionState`:
+ `IDLE`: the gripper is not moving, or no detected objects exist;
+ `UNLOCKED`: the gripper is moving, but the target object is unknown;
+ `LOCKED`: the gripper is moving toward one object with sufficient confidence, so the system provides assistance to the gripper;
+ `RETREATED`: action to leave `LOCKED` state while the assistance provided by the system is out of anticipation, and this action takes some cooldown time before transferring to `UNLOCKED` state;
+ `REACHED`: the gripper has reached the target object;
+ `GRASPED`: the gripper is closed, often when grasping the object;

State transition (listed by priority):
+ ALL -> `IDLE` when...
  - the positional safety button is not pressed, or
  - no detected objects exist.
+ ALL -> `GRASPED` when...
  - not in `IDLE` state, and
  - the gripper is closed
+ ALL -> `REACHED` when...
  - not int `IDLE` or `GRASPED` state, and
  - the gripper is near the target (distance less than 3 cm)
  - [Exception] in `LOCKED` state and the planned position has not been reached
+ `IDLE` -> ...
  - `RETREATED` when the retreating cooldown has not finished
  - `UNLOCKED` otherwise
+ `UNLOCKED` -> `LOCKED` when...
  - the gripper is moving toward some object (angle between `user_command_direction` and `object_direction` is less than 45 degrees), 
  - the object has sufficient confidence (`confidence` > 0.3), and
  - the above conditions are satisfied for at least 0.5 seconds
+ `LOCKED` -> `RETREATED` when the gripper is moving away from the target object (angle between `user_command_direction` and `target_direction` is more than 90 degrees)
+ `RETREATED` -> `UNLOCKED` when the cooldown has finished


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