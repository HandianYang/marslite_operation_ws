#include "cartesian_control/cartesian_control_keyboard.h"
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

CartesianControlKeyboard::CartesianControlKeyboard(const ros::NodeHandle& nh)
  : nh_(nh), input_key_('\0'), rate_(ros::Rate(60)), is_stopped_(false)
{
  target_frame_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_frame", 10);

  current_pose_.header.frame_id = "base_link";
  current_pose_.pose.position.x = 0.712;
  current_pose_.pose.position.y = -0.122;
  current_pose_.pose.position.z = 1.043;
  current_pose_.pose.orientation.x = 0.5;
  current_pose_.pose.orientation.y = 0.5;
  current_pose_.pose.orientation.z = 0.5;
  current_pose_.pose.orientation.w = 0.5;
  target_frame_pub_.publish(current_pose_);
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
  switch (input_key_)
  {
    case 'w':
    case 'W':
      current_pose_.pose.position.x += 0.01;
      break;
    case 's':
    case 'S':
      current_pose_.pose.position.x -= 0.01;
      break;
    case 'a':
    case 'A':  
      current_pose_.pose.position.y += 0.01;
      break;
    case 'd':
    case 'D':
      current_pose_.pose.position.y -= 0.01;
      break;
    case 'r':
    case 'R':
      current_pose_.pose.position.z += 0.01;
      break;
    case 'f':
    case 'F':
      current_pose_.pose.position.z -= 0.01;
      break;
    case 'q':
    case 'Q':
      is_stopped_ = true;
      break;
  }
}

void CartesianControlKeyboard::publishPose()
{
  target_frame_pub_.publish(current_pose_);
}