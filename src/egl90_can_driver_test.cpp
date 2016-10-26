#include "ros/ros.h"
#include "schunk_gripper_egl90/egl90_can_node.h"
#include <iostream>
#include <boost/progress.hpp>
#include <boost/timer.hpp>

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

	const unsigned int totalTestCycles	= 1000;
    boost::progress_display show_progress(totalTestCycles);
    // Insert here the test commands.
    for(int i = 0; i <= totalTestCycles; i++)
    {
        //ROS_INFO("Cycle No. %d", i);
		//std::cout<<"Cycle No. "<<i<<"/1000"<<std::endl;
		//ros::Duration(3).sleep();
		move_grip(-20.0, 1.0);
		//acknowledge();
		//ros::Duration(3).sleep();
		//move_grip(20.0, 1.0);
		//acknowledge();
		//move_pos(10);
		move_pos(68.0);
		move_pos(50.0);
		move_pos(0.0);
		move_pos(50);
		//ros::Duration(3).sleep();
        ++show_progress;
    }
}

int acknowledge()
{
	ros::NodeHandle n_ack;
	ros::ServiceClient c_ack = n_ack.serviceClient<std_srvs::Trigger>("schunk_gripper_egl90_node/acknowledge");
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
	ros::ServiceClient c_ref_mot = n_ref_mot.serviceClient<std_srvs::Trigger>("schunk_gripper_egl90_node/reference_motion");
	std_srvs::Trigger srv;
	if (c_ref_mot.call(srv))
	{
	  ROS_DEBUG("Moving to reference position.");
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
	ros::ServiceClient c_move_pos = n_move_pos.serviceClient<schunk_gripper_egl90::MovePos>("schunk_gripper_egl90_node/move_pos");
	schunk_gripper_egl90::MovePos srv;
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
	ros::ServiceClient c_move_grip = n_move_grip.serviceClient<schunk_gripper_egl90::MoveGrip>("schunk_gripper_egl90_node/move_grip");
	schunk_gripper_egl90::MoveGrip srv;
	srv.request.speed = sp;
	srv.request.current = cu;
	if (c_move_grip.call(srv))
	{
	  ROS_DEBUG("Moving the gripper fingers.");
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
	ros::ServiceClient c_clean = n_clean.serviceClient<std_srvs::Trigger>("schunk_gripper_egl90_node/clean_up");
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
	ros::ServiceClient c_stop = n_stop.serviceClient<std_srvs::Trigger>("schunk_gripper_egl90_node/stop");
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
