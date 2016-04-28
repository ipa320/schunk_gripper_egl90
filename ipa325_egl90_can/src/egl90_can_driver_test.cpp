#include "ros/ros.h"
#include "ipa325_egl90_can/egl90_can_node.h"

int acknowledge(void);
int reference_motion(void);
int move_pos(float);
int move_grip(float, float);
int clean_up(void);
int stop(void);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "egl90_can_driver_test");
	if (argc != 1)
	{
		ROS_INFO("Usage: egl90_can_driver_test (No arguments)");
		return 1;
	}

	// Insert here the test commands.

	move_pos(10);
	ros::Duration(3).sleep();
	move_pos(20);
	ros::Duration(3).sleep();
	move_pos(10);
	ros::Duration(3).sleep();
	move_pos(20);
	ros::Duration(3).sleep();
	move_grip(10, 0.1);
	ros::Duration(3).sleep();
	move_grip(-10, 0.1);
	ros::Duration(3).sleep();
	move_grip(10, 0.1);
	ros::Duration(3).sleep();
	move_grip(10, 0.1);
	ros::Duration(3).sleep();
	acknowledge();
}

int acknowledge()
{
	ros::NodeHandle n_ack;
	ros::ServiceClient c_ack = n_ack.serviceClient<std_srvs::Trigger>("ipa325_egl90_can_node/acknowledge");
	std_srvs::Trigger srv;
	if (c_ack.call(srv))
	{
	  ROS_INFO("Acknowledging.");
	}
	else
	{
	  ROS_ERROR("Failed to call service acknowledge");
	  return 1;
	}
}

int reference_motion()
{
	ros::NodeHandle n_ref_mot;
	ros::ServiceClient c_ref_mot = n_ref_mot.serviceClient<std_srvs::Trigger>("ipa325_egl90_can_node/reference_motion");
	std_srvs::Trigger srv;
	if (c_ref_mot.call(srv))
	{
	  ROS_INFO("Moving to reference position.");
	}
	else
	{
	  ROS_ERROR("Failed to call service reference motion");
	  return 1;
	}
}

int move_pos(float pos)
{
	ros::NodeHandle n_move_pos;
	ros::ServiceClient c_move_pos = n_move_pos.serviceClient<ipa325_egl90_can::MovePos>("ipa325_egl90_can_node/move_pos");
	ipa325_egl90_can::MovePos srv;
	srv.request.position = pos;
	if (c_move_pos.call(srv))
	{
	  ROS_INFO("Moving to an specific position.");
	}
	else
	{
	  ROS_ERROR("Failed to call service move_grip");
	  return 1;
	}
}

int move_grip(float sp, float cu)
{
	ros::NodeHandle n_move_grip;
	ros::ServiceClient c_move_grip = n_move_grip.serviceClient<ipa325_egl90_can::MoveGrip>("ipa325_egl90_can_node/move_grip");
	ipa325_egl90_can::MoveGrip srv;
	srv.request.speed = sp;
	srv.request.current = cu;
	if (c_move_grip.call(srv))
	{
	  ROS_INFO("Moving the gripper fingers.");
	}
	else
	{
	  ROS_ERROR("Failed to call service move_grip");
	  return 1;
	}
}

int clean_up()
{
	ros::NodeHandle n_clean;
	ros::ServiceClient c_clean = n_clean.serviceClient<std_srvs::Trigger>("ipa325_egl90_can_node/clean_up");
	std_srvs::Trigger srv;
	if (c_clean.call(srv))
	{
	  ROS_INFO("Cleaning up.");
	}
	else
	{
	  ROS_ERROR("Failed to call service clean_up");
	  return 1;
	}
}

int stop()
{
	ros::NodeHandle n_stop;
	ros::ServiceClient c_stop = n_stop.serviceClient<std_srvs::Trigger>("ipa325_egl90_can_node/stop");
	std_srvs::Trigger srv;
	if (c_stop.call(srv))
	{
	  ROS_INFO("Stoping.");
	}
	else
	{
	  ROS_ERROR("Failed to call service stop");
	  return 1;
	}
}
