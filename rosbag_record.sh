# rosbag_record.sh
#!/usr/bin/env bash

rosbag record \
  -O "bug_$(date +%F_%H%M%S).bag" \
  --lz4 --split --size=2048 \
  /tf /tf_static \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /unity/controller/left/pose \
  /unity/controller/left/joy \
  /cartesian_control/target_frame \
  /marslite_control/gripper_pose \
  /marslite_control/user_desired_gripper_pose \
  /marslite_control/user_desired_gripper_status \
  /marslite_control/user_command_velocity \
  /marslite_control/position_safety_button_signal \
  /marslite_control/gripper_motion_state