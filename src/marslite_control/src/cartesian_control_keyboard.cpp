#include "cartesian_control/cartesian_control_keyboard.h"
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

CartesianControlKeyboard::CartesianControlKeyboard(const ros::NodeHandle& nh)
  : nh_(nh), input_key_('\0'), rate_(ros::Rate(60)), is_stopped_(false), is_position_control_(true)
{
  target_frame_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_frame", 10);
  target_pose_ = kInitialGripperPose;
  ROS_INFO_STREAM("Switch to position control mode.");
}

void CartesianControlKeyboard::run()
{
  while (ros::ok() && !is_stopped_) {
    getKeyboardInput();
    if (input_key_ != '\0') {
      processInput();
      publishPose();
    }
    
    rate_.sleep();
    ros::spinOnce();
  }
}

RPY CartesianControlKeyboard::convertQuaternionMsgToRPY(const geometry_msgs::Quaternion& quaternion)
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

geometry_msgs::Quaternion CartesianControlKeyboard::convertRPYToQuaternionMsg(const RPY& rpy)
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

void CartesianControlKeyboard::getKeyboardInput()
{
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

void CartesianControlKeyboard::processInput()
{
  static ros::Time last_switch_time = ros::Time(0);
  switch (input_key_)
  {
    // Q/q: Stop the program
    case 'q':
    case 'Q':
      is_stopped_ = true;
      break;
    // E/e: Switch between position and orientation control
    case 'e':
    case 'E':
      if ((ros::Time::now() - last_switch_time).toSec() >= 0.5) {
        is_position_control_ = !is_position_control_;
        last_switch_time = ros::Time::now();
        if (is_position_control_) {
          ROS_INFO_STREAM("Switch to position control mode.");
        } else {
          ROS_INFO_STREAM("Switch to orientation control mode.");
          target_pose_rpy_ = this->convertQuaternionMsgToRPY(target_pose_.pose.orientation);
        }
      }
      break;
    // space: Reset the gripper pose to the initial pose
    case ' ':
      target_pose_ = kInitialGripperPose;
      target_pose_rpy_ = this->convertQuaternionMsgToRPY(target_pose_.pose.orientation);
      break;
    default:
      if (is_position_control_) {
        processInputInPositionControl();
      } else {
        processInputInOrientationControl();
      }
      break;
  }
}

void CartesianControlKeyboard::processInputInPositionControl()
{
  switch (input_key_)
  {
    case 'w':
    case 'W':
      target_pose_.pose.position.x += 0.01;
      break;
    case 's':
    case 'S':
      target_pose_.pose.position.x -= 0.01;
      break;
    case 'a':
    case 'A':
      target_pose_.pose.position.y += 0.01;
      break;
    case 'd':
    case 'D':
      target_pose_.pose.position.y -= 0.01;
      break;
    case 'r':
    case 'R':
      target_pose_.pose.position.z += 0.01;
      break;
    case 'f':
    case 'F':
      target_pose_.pose.position.z -= 0.01;
      break;
  }
}

void CartesianControlKeyboard::processInputInOrientationControl()
{
  switch (input_key_)
  {
    case 'w':
    case 'W':
      // target_pose_rpy_.roll += M_PI / 180.0;
      target_pose_rpy_.pitch -= M_PI / 180.0;
      break;
    case 's':
    case 'S':
      // target_pose_rpy_.roll -= M_PI / 180.0;
      target_pose_rpy_.pitch += M_PI / 180.0;
      break;
    case 'a':
    case 'A':
      // target_pose_rpy_.pitch += M_PI / 180.0;
      target_pose_rpy_.roll += M_PI / 180.0;
      break;
    case 'd':
    case 'D':
      // target_pose_rpy_.pitch -= M_PI / 180.0;
      target_pose_rpy_.roll -= M_PI / 180.0;
      break;
    case 'r':
    case 'R':
      target_pose_rpy_.yaw += M_PI / 180.0;
      break;
    case 'f':
    case 'F':
      target_pose_rpy_.yaw -= M_PI / 180.0;
      break;
  }

  target_pose_.pose.orientation = convertRPYToQuaternionMsg(target_pose_rpy_);
}

void CartesianControlKeyboard::publishPose()
{
  target_frame_pub_.publish(target_pose_);
}