#include "ros/ros.h"
#include "schunk_gripper_egl90/egl90_can_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "schunk_gripper_egl90_node");

  ROS_INFO("Starting EGL090 CAN Node.");
  Egl90_can_node* node = new Egl90_can_node();
  ROS_INFO("EGL090 CAN Node running...");

  node->spin();

  return 0;
}
