#include "cartesian_control/keyboard_teleoperation.h"

#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


/******************************************************
 *                  Constructors                      *  
 ****************************************************** */

KeyboardTeleoperation::KeyboardTeleoperation(const ros::NodeHandle& nh)
    : nh_(nh), input_key_('\0'), rate_(ros::Rate(25)),
      is_stopped_(false), is_position_control_(true) {
  this->parseParameters();
  this->initializePublishers();
  this->setInitialGripperPose();

  ROS_INFO_STREAM("Switch to position control mode.");
}

/******************************************************
 *                  Public members                    *  
 ****************************************************** */

void KeyboardTeleoperation::run() {
  while (nh_.ok() && !is_stopped_) {
    this->getKeyboardInput();
    if (input_key_ != '\0' && input_key_ != 'q' && input_key_ != 'Q') {
      this->processInput();
      this->publishPose();
    }
    
    rate_.sleep();
    ros::spinOnce();
  }
}

/******************************************************
 *                  Private members                   *  
 ****************************************************** */

void KeyboardTeleoperation::parseParameters() {
  ros::NodeHandle pnh("~");
  pnh.param("use_sim", use_sim_, false);
  ROS_INFO_STREAM(std::boolalpha << "Parameters: "
      << "\n * use_sim: " << use_sim_
  );
}

void KeyboardTeleoperation::initializePublishers() {
  const std::string gripper_pose_topic = "/target_frame";
  const std::string gripper_status_topic = "/gripper/cmd_gripper";
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(gripper_pose_topic, 1);
  desired_gripper_status_publisher_ = nh_.advertise<std_msgs::Bool>(gripper_status_topic, 1);
}

void KeyboardTeleoperation::setInitialGripperPose() {
  const std::string source_frame = "base_link";
  const std::string target_frame = use_sim_ ? "robotiq_85_base_link" : "tm_gripper";
  geometry_msgs::TransformStamped gripper_transform = 
      tf2_listener_.lookupTransform(source_frame, target_frame);
  desired_gripper_pose_.header.frame_id = "base_link";
  desired_gripper_pose_.pose.position.x = gripper_transform.transform.translation.x;
  desired_gripper_pose_.pose.position.y = gripper_transform.transform.translation.y;
  desired_gripper_pose_.pose.position.z = gripper_transform.transform.translation.z;
  desired_gripper_pose_.pose.orientation = gripper_transform.transform.rotation;
}

void KeyboardTeleoperation::getKeyboardInput() {
  struct termios oldt, newt;
  input_key_ = '\0';

  // Get the file descriptor for the standard input (keyboard)
  int fd = fileno(stdin);

  // Save the current terminal settings
  tcgetattr(fd, &oldt);
  newt = oldt;

  // Set the terminal to raw mode
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(fd, TCSANOW, &newt);

  // Set non-blocking mode on stdin
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  // Check if there is input available
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;  // 100 ms timeout

  FD_ZERO(&fds);
  FD_SET(fd, &fds);

  int selectRet = select(fd + 1, &fds, NULL, NULL, &tv);
  if (selectRet > 0 && FD_ISSET(fd, &fds))
  {
    input_key_ = getchar(); // Read a character from the keyboard
  }

  // Restore the terminal settings
  tcsetattr(fd, TCSANOW, &oldt);
}

void KeyboardTeleoperation::processInput() {
  static ros::Time last_switch_time = ros::Time(0);
  switch (input_key_) {
    // Q/q: Stop the program
    case 'q':
    case 'Q':
      is_stopped_ = true;
      ROS_INFO_STREAM("Terminate the program.");
      break;
    // E/e: Switch between position and orientation control
    case 'e':
    case 'E':
      // Add 0.5 seconds cooldown to prevent rapid switching
      if ((ros::Time::now() - last_switch_time).toSec() >= 0.5) {
        is_position_control_ = !is_position_control_;
        last_switch_time = ros::Time::now();
        if (is_position_control_) {
          ROS_INFO_STREAM("Switch to position control mode.");
        } else {
          desired_gripper_pose_rpy_.convertFromPoseStamped(desired_gripper_pose_);
          ROS_INFO_STREAM("Switch to orientation control mode.");
        }
      }
      break;
    // space: Reset the gripper pose to the initial pose
    case ' ':
      desired_gripper_pose_ = use_sim_ ?
          marslite::sim::kFrontInitialGripperPose :
          marslite::real::kFrontInitialGripperPose;
      desired_gripper_pose_rpy_.convertFromPoseStamped(desired_gripper_pose_);
      ROS_INFO_STREAM("Reset to the initial gripper pose.");
      break;
    case 'g':
    case 'G':
      desired_gripper_status_.data = !desired_gripper_status_.data;
      desired_gripper_status_publisher_.publish(desired_gripper_status_);
      break;
    default:
      if (is_position_control_) {
        this->processInputInPositionControl();
      } else {
        this->processInputInOrientationControl();
      }
      break;
  }
}

void KeyboardTeleoperation::processInputInPositionControl() {
  switch (input_key_) {
    case 'w':
    case 'W':
      desired_gripper_pose_.pose.position.x += 0.001;
      break;
    case 's':
    case 'S':
      desired_gripper_pose_.pose.position.x -= 0.001;
      break;
    case 'a':
    case 'A':
      desired_gripper_pose_.pose.position.y += 0.001;
      break;
    case 'd':
    case 'D':
      desired_gripper_pose_.pose.position.y -= 0.001;
      break;
    case 'r':
    case 'R':
      desired_gripper_pose_.pose.position.z += 0.001;
      break;
    case 'f':
    case 'F':
      desired_gripper_pose_.pose.position.z -= 0.001;
      break;
  }
}

void KeyboardTeleoperation::processInputInOrientationControl() {
  // [NOTE] The transformation from `/base_link` to `/tm_gripper` is applied here`:
  //   | /base_link | /tm_gripper |
  //   |------------|-------------|
  //   |   +-roll   |   -+pitch   |
  //   |   +-pitch  |   +-roll    |
  //   |   +-yaw    |   +-yaw     |
  switch (input_key_) {
    case 'w':
    case 'W':
      desired_gripper_pose_rpy_.pitch -= M_PI / 180.0;
      break;
    case 's':
    case 'S':
      desired_gripper_pose_rpy_.pitch += M_PI / 180.0;
      break;
    case 'a':
    case 'A':
      desired_gripper_pose_rpy_.roll += M_PI / 180.0;
      break;
    case 'd':
    case 'D':
      desired_gripper_pose_rpy_.roll -= M_PI / 180.0;
      break;
    case 'r':
    case 'R':
      desired_gripper_pose_rpy_.yaw += M_PI / 180.0;
      break;
    case 'f':
    case 'F':
      desired_gripper_pose_rpy_.yaw -= M_PI / 180.0;
      break;
  }

  desired_gripper_pose_.pose.orientation = desired_gripper_pose_rpy_.convertToQuaternion();
}