schunk_gripper_egl90
=========================================

ROS Driver for the EGL90 Gripper by Schunk based on CAN bus and Schunk-Motion-Interface

## Usage

```rosrun schunk_gripper_egl90 schunk_gripper_egl90_node ```

All capabilities are available via ros-service calls:
 * std_srvs::Trigger : /acknowledge
 * std_srvs::Trigger : /reference_motion
 * schunk_gripper_egl90::MovePos : /move_pos
 * schunk_gripper_egl90::MoveGrip : /move_grip
 * std_srvs::Trigger : /clean_up
 * std_srvs::Trigger : /stop

## Acknowledgements
This project is a result of the LIAA project.
http://www.project-leanautomation.eu/

![LIAA](http://www.project-leanautomation.eu/fileadmin/img/LIAALogo/Logo_LIAA.png "LIAA")

![EC](http://www.project-leanautomation.eu/typo3temp/pics/b3ba71db31.jpg "EC")

LIAA received funding from the European Union’s Seventh Framework Programme for research, technological development and demonstration under grant agreement no. 608604.

Project runtime: 02.09.2013 – 31.08.2017.