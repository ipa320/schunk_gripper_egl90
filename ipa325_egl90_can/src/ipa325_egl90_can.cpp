#include "ipa325_egl90_can/egl90_can_node.h"

#include <signal.h>
#include <sensor_msgs/JointState.h>

#include <socketcan_interface/string.h>

bool Egl90_can_node::_shutdownSignal = false;

Egl90_can_node::Egl90_can_node()
{
    //signal handler
    signal((int) SIGINT, Egl90_can_node::signalHandler);

    _nh = ros::NodeHandle("~");
    std::string nodename = ros::this_node::getName();

    _srv_ack = _nh.advertiseService(nodename+"/acknowledge", &Egl90_can_node::acknowledge, this);
    _srv_reference = _nh.advertiseService(nodename+"/reference_motion", &Egl90_can_node::moveToReferencePos, this);
    _srv_movePos = _nh.advertiseService(nodename+"/move_pos", &Egl90_can_node::movePos, this);
    _srv_moveGrip = _nh.advertiseService(nodename+"/move_grip", &Egl90_can_node::moveGrip, this);
    _srv_getState = _nh.advertiseService(nodename+"/get_state", &Egl90_can_node::getState, this);
    _srv_stop = _nh.advertiseService(nodename+"/stop", &Egl90_can_node::stop, this);

    _pub_joint_states = _nh.advertise<sensor_msgs::JointState>("joint_states", 1000);


    // TODO: Make this parameters
    _can_id = 0x050C; // 0x05 for master, module id 0xC = 12
    _can_module_id = 0x070C; // 0x07 for slave, module id 0xC = 12
    _can_socket_id = "can0"; // name within linux ifconfig


    if(!_can_driver.init(_can_socket_id, false)) // read own messages: false
    {
        ROS_ERROR("Error in can socket initialization");
        exit(-2);
    }
    // read own messages: false
    can::CommInterface::FrameListener::Ptr _respListener = _can_driver.createMsgListener(can::MsgHeader(_can_module_id), &Egl90_can_node::handleFrame_response);

     ROS_INFO("Can socket binding was successful!");
     acknowledge();
     updateState();

    _timer = _nh.createTimer(ros::Duration(1.0/50.0), &Egl90_can_node::timer_cb, this);

}

void Egl90_can_node::handleFrame_response(const can::Frame &f)
{

}

void Egl90_can_node::timer_cb(const ros::TimerEvent&)
{
    publishState();
}

bool Egl90_can_node::moveToReferencePos(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
//    struct can_frame txframe, rxframe;
//    bool error_flag = false;

//    txframe.can_id = _can_id;
//    txframe.can_dlc = 0x02;
//    txframe.data[0] = 0x01;
//    txframe.data[1] = 0x92; //CMD Byte

    _can_driver.send(can::toframe("7#0192"));
    return true;
}

void Egl90_can_node::acknowledge()
{
    _can_driver.send(can::toframe("7#018B"));
}

bool Egl90_can_node::acknowledge(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    struct can_frame txframe, rxframe;
    bool error_flag = false;

    txframe.can_id = _can_id;
    txframe.can_dlc = 0x02;
    txframe.data[0] = 0x01;
    txframe.data[1] = 0x8B; //CMD Byte

//    write(_can_socket, &txframe, sizeof(struct can_frame));
//    do
//    {
//        ros::Duration(0.01).sleep();
//        read(_can_socket, &rxframe, sizeof(struct can_frame));
//    } while (!isCanAnswer(0x8B, rxframe, error_flag) || _shutdownSignal);

//    if (error_flag)
//    {
//        res.success = false;
//        res.message = "Module did reply with error 0x02!";
//    }
//    else
//    {
//        res.success = true;
//        res.message = "Module did reply properly!";
//    }

//    updateState();
    return true;
}

bool Egl90_can_node::stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    struct can_frame txframe, rxframe;
    bool error_flag = false;

    txframe.can_id = _can_id;
    txframe.can_dlc = 0x02;
    txframe.data[0] = 0x01;
    txframe.data[1] = 0x91;

//    write(_can_socket, &txframe, sizeof(struct can_frame));
//    do
//    {
//        ros::Duration(0.01).sleep();
//        read(_can_socket, &rxframe, sizeof(struct can_frame));
//    } while (!isCanAnswer(0x91, rxframe, error_flag) || _shutdownSignal);

//    if (error_flag)
//    {
//        res.success = false;
//        res.message = "Module did reply with error 0x02!";
//    }
//    else
//    {
//        res.success = true;
//        res.message = "Module did reply properly!";
//    }

//    updateState();
    return true;
}

statusData Egl90_can_node::updateState()
{
    struct can_frame txframe, rxframe1, rxframe2, rxframe3;

    txframe.can_id = _can_id;
    txframe.can_dlc = 0x02;
    txframe.data[0] = 0x01;
    txframe.data[1] = 0x95;//CMD Byte

//    write(_can_socket, &txframe, sizeof(struct can_frame));

//    ros::Duration(0.01).sleep();
//    // TODO the fragmented CAN message protocol is weird, this will only work if the can is empty besided this module
//    read(_can_socket, &rxframe1, sizeof(struct can_frame));
//    read(_can_socket, &rxframe2, sizeof(struct can_frame));
//    read(_can_socket, &rxframe3, sizeof(struct can_frame));

    statusData status;
//    status.c[0] = rxframe1.data[3];
//    status.c[1] = rxframe1.data[4];
//    status.c[2] = rxframe1.data[5];
//    status.c[3] = rxframe1.data[6];
//    status.c[4] = rxframe1.data[7];

//    status.c[5] = rxframe2.data[2];
//    status.c[6] = rxframe2.data[3];
//    status.c[7] = rxframe2.data[4];
//    status.c[8] = rxframe2.data[5];
//    status.c[9] = rxframe2.data[6];
//    status.c[10] = rxframe2.data[7];

//    status.c[11] = rxframe3.data[2];
//    status.c[12] = rxframe3.data[3];
//    status.c[13] = rxframe3.data[4];

//    ROS_INFO("Position: %f,\nVelocity: %f,\nCurrent: %f", status.status.position, status.status.speed, status.status.current);

//    ROS_WARN("Status bits may not correct!");
//    ROS_INFO("IsReferenced: %s,\nIsMoving: %s,\nIsInProgMode: %s\nIsWarning: %s\nIsError: %s\nIsBraked: %s\nisMotionInterrupted: %s\nIsTargetReached: %s\nErrorCode: %X",
//             (status.status.statusBits >> 0) & 1 ? "True" : "False",
//             (status.status.statusBits >> 1) & 1 ? "True" : "False",
//             (status.status.statusBits >> 2) & 1 ? "True" : "False",
//             (status.status.statusBits >> 3) & 1 ? "True" : "False",
//             (status.status.statusBits >> 4) & 1 ? "True" : "False",
//             (status.status.statusBits >> 5) & 1 ? "True" : "False",
//             (status.status.statusBits >> 6) & 1 ? "True" : "False",
//             (status.status.statusBits >> 7) & 1 ? "True" : "False",
//             status.status.errorCode);

    // Convert to meter
    status.status.position *= 0.001;
    status.status.speed *= 0.001;
    _status = status;
    return status;
}

bool Egl90_can_node::getState(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.success = true;
    res.message = "ok";

    updateState();
    return true;
}

bool Egl90_can_node::publishState()
{
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    js.name.push_back("egl_position");
    js.position.push_back(_status.status.position);
    js.velocity.push_back(_status.status.speed);
    js.effort.push_back(_status.status.current);
    _pub_joint_states.publish(js);
}

bool Egl90_can_node::movePos(ipa325_egl90_can::MovePos::Request &req, ipa325_egl90_can::MovePos::Response &res)
{
    fdata pos;
    pos.f = req.position;
    bool error_flag = false;

    struct can_frame txframe, rxframe;

    txframe.can_id = _can_id;
    txframe.can_dlc = 6;

    txframe.data[0] = 5;
    txframe.data[1] = 0xB0; //CMD Byte

    txframe.data[2] = pos.c[0];
    txframe.data[3] = pos.c[1];
    txframe.data[4] = pos.c[2];
    txframe.data[5] = pos.c[3];

//    write(_can_socket, &txframe, sizeof(struct can_frame));
//    do
//    {
//        ros::Duration(0.01).sleep();
//        read(_can_socket, &rxframe, sizeof(struct can_frame));
//    } while (!isCanAnswer(0xB0, rxframe, error_flag) || _shutdownSignal);

//    if (error_flag)
//    {
//        res.success = false;
//        res.message = "Module did reply with error 0x02!";
//    }
//    else
//    {
//        do // wait for position reached signal
//        {
//            ros::Duration(0.01).sleep();
//            read(_can_socket, &rxframe, sizeof(struct can_frame));
//        } while ((rxframe.can_dlc < 2 && rxframe.data[2] != 0x94) || _shutdownSignal); // 0x94 is CMD_POS_REACHED

//        // TODO: timeout while reading socket
//        bool timeout = false;
//        if (timeout)
//        {
//            res.success = false;
//            res.message = "Module did not reply properly!";
//            return true;
//        }
//        else
//        {
//            res.success = true;
//            res.message = "Module reached position!";
//        }
//    }

//    updateState();
    return true;
}

bool Egl90_can_node::moveGrip(ipa325_egl90_can::MoveGrip::Request &req, ipa325_egl90_can::MoveGrip::Response &res)
{
     ROS_INFO("move_grip seems not to be available in module 12, this is the alternativ implementation using velocity cmd and underlying current control!");

     ROS_WARN("The parameter you give in this command will be the default parameters for future move_pos commands!");

     fdata vel, cur;
     bool error_flag = false;

     vel.f = req.speed;
     cur.f = req.current;
     struct can_frame tx1frame, tx2frame, rxframe;

     tx1frame.can_id = _can_id;
     tx2frame.can_id = _can_id;
     tx1frame.can_dlc = 8;

     tx1frame.data[0] = 9; // DLEN Total Data length to come (1Byte CMD + 4Byte Vel + 4Byte Cur)
     tx1frame.data[1] = 0x84; // First fragment (Fragmentsmarker do not count in DLEN)
     tx1frame.data[2] = 0xB5; // Move_vel

     tx1frame.data[3] = vel.c[0];
     tx1frame.data[4] = vel.c[1];
     tx1frame.data[5] = vel.c[2];
     tx1frame.data[6] = vel.c[3];

     tx1frame.data[7] = cur.c[0];

     tx2frame.data[0] = 3;
     tx2frame.data[1] = 0x86; //Last fragment
     tx2frame.data[2] = cur.c[1];
     tx2frame.data[3] = cur.c[2];
     tx2frame.data[4] = cur.c[3];
     tx2frame.can_dlc = 5;

//     write(_can_socket, &tx1frame, sizeof(struct can_frame));
//     write(_can_socket, &tx2frame, sizeof(struct can_frame));

//     do
//     {
//         ros::Duration(0.01).sleep();
//         read(_can_socket, &rxframe, sizeof(struct can_frame));
//     } while (!isCanAnswer(0xB5, rxframe, error_flag) || _shutdownSignal);

//     if (error_flag)
//     {
//         res.success = false;
//         res.message = "Module did reply with error 0x02!";
//     }
//     else
//     {
//         do // wait for motion blocked signal
//         {
//             ros::Duration(0.01).sleep();
//             read(_can_socket, &rxframe, sizeof(struct can_frame));
//         } while ((rxframe.can_dlc < 2 && rxframe.data[2] != 0x93) || _shutdownSignal); // 0x93 is CMD_MOVE_BLOCKED

//         // TODO: timeout while reading socket
//         bool timeout = false;
//         if (timeout)
//         {
//             res.success = false;
//             res.message = "Module did not reply properly!";
//             return true;
//         }
//         else
//         {
//             res.success = true;
//             res.message = "Module reached position!";
//         }
//     }

//     updateState();
     return true;
     // --------------move grip --------------------------//
/*     fdata cur;
     cur.f = req.current;
     bool error_flag = false;

     struct can_frame txframe, rxframe;

     txframe.can_id = _can_id;
     txframe.can_dlc = 6;

     txframe.data[0] = 5;
     txframe.data[1] = 0xB7; // Move_grip seems to be not available in module 12

     txframe.data[2] = cur.c[0];
     txframe.data[3] = cur.c[1];
     txframe.data[4] = cur.c[2];
     txframe.data[5] = cur.c[3];

     write(_can_socket, &txframe, sizeof(struct can_frame));
     do
     {
         ros::Duration(0.01).sleep();
         read(_can_socket, &rxframe, sizeof(struct can_frame));
     } while (!isCanAnswer(0xB7, rxframe, error_flag) || _shutdownSignal);

     if (error_flag)
     {
         res.success = false;
         res.message = "Module did reply with error 0x02!";
     }
     else
     {
         do // wait for position reached signal
         {
             ros::Duration(0.01).sleep();
             read(_can_socket, &rxframe, sizeof(struct can_frame));
         } while ((rxframe.can_dlc < 2 && rxframe.data[2] != 0x93) || _shutdownSignal); // 0x93 is CMD_MOVE_BLOCKED

         // TODO: timeout while reading socket
         bool timeout = false;
         if (timeout)
         {
             res.success = false;
             res.message = "Module did not reply properly!";
             return true;
         }
         else
         {
             res.success = true;
             res.message = "Module reached position!";
         }
     }

     return true;
*/
}

bool Egl90_can_node::isCanAnswer(unsigned int cmd, const can_frame& rxframe, bool& error_flag)
{
    error_flag = (rxframe.data[0] == 0x02);
    return (rxframe.can_id == _can_module_id &&
            rxframe.data[1] == cmd);
}

void Egl90_can_node::spin()
{
    ros::Rate rate(100);

    //wait for shutdown
    while(ros::ok() && !Egl90_can_node::_shutdownSignal)
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void Egl90_can_node::signalHandler(int signal)
{
    ROS_INFO_NAMED("driver", "shutdown");
    _shutdownSignal = true;
}

