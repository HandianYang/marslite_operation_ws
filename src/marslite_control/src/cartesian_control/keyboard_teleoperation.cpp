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

void KeyboardTeleoperation::teleoperate() {
  while (nh_.ok() && !is_stopped_) {
    this->getKeyboardInput();
    if (input_key_ != '\0') {
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
  pnh.param("use_shared_controller", use_shared_controller_, false);
  ROS_INFO_STREAM("Parameters: "
      << "\n * use_shared_controller: " << use_shared_controller_);
}

void KeyboardTeleoperation::initializePublishers() {
  const std::string gripper_pose_topic = use_shared_controller_ ?
      "/cartesian_control/user_desired_gripper_pose" :
      "/cartesian_control/target_frame";
  const std::string gripper_status_topic = use_shared_controller_ ?
      "/cartesian_control/user_desired_gripper_status" :
      "/gripper/cmd_gripper";
  desired_gripper_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(gripper_pose_topic, 1);
  desired_gripper_status_publisher_ = nh_.advertise<std_msgs::Bool>(gripper_status_topic, 1);
}

void KeyboardTeleoperation::setInitialGripperPose() {
  desired_gripper_pose_ = kInitialGripperPose;
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
          desired_gripper_pose_rpy_ = this->convertQuaternionMsgToRPY(desired_gripper_pose_.pose.orientation);
          ROS_INFO_STREAM("Switch to orientation control mode.");
        }
      }
      break;
    // space: Reset the gripper pose to the initial pose
    case ' ':
      desired_gripper_pose_ = kInitialGripperPose;
      desired_gripper_pose_rpy_ = this->convertQuaternionMsgToRPY(desired_gripper_pose_.pose.orientation);
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

void KeyboardTeleoperation::publishPose() {
  desired_gripper_pose_publisher_.publish(desired_gripper_pose_);
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

void KeyboardTeleoperation::processInputInOrientationControl()
{
  // [NOTE] The transformation from `/base_link` to `/tm_gripper` is applied here`:
  //   | /base_link | /tm_gripper |
  //   |------------|-------------|
  //   |   +-roll   |   -+pitch   |
  //   |   +-pitch  |   +-roll    |
  //   |   +-yaw    |   +-yaw     |
  switch (input_key_)
  {
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

  desired_gripper_pose_.pose.orientation = convertRPYToQuaternionMsg(desired_gripper_pose_rpy_);
}

RPY KeyboardTeleoperation::convertQuaternionMsgToRPY(const geometry_msgs::Quaternion& quaternion)
{
  RPY rpy;
  tf2::Quaternion q(
      quaternion.x,
      quaternion.y,
      quaternion.z,
      quaternion.w
  );
  if (q.length2() == 0.0) {
    ROS_ERROR("Invalid quaternion with zero length encountered in getRPYFromPose.");
    rpy.roll = rpy.pitch = rpy.yaw = 0.0;
    return rpy;
  }

  q.normalize();
  tf2::Matrix3x3 m(q);
  m.getRPY(rpy.roll, rpy.pitch, rpy.yaw);

  return rpy;
}

geometry_msgs::Quaternion KeyboardTeleoperation::convertRPYToQuaternionMsg(const RPY& rpy)
{
  tf2::Quaternion q;
  q.setRPY(rpy.roll, rpy.pitch, rpy.yaw);
  q.normalize();

  geometry_msgs::Quaternion quaternion_msg;
  quaternion_msg.x = q.x();
  quaternion_msg.y = q.y();
  quaternion_msg.z = q.z();
  quaternion_msg.w = q.w();

  return quaternion_msg;
}