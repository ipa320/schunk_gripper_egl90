#include "ros/ros.h"
#include "ipa325_egl90_can/egl90_can_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ipa325_egl_can_node");

  Egl90_can_node* node = new Egl90_can_node();
  ROS_INFO("Starting EGL090 CAN Node.");

  node->spin();

  return 0;
}
