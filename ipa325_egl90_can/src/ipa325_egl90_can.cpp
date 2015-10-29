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
    _module_adress = 12;
    _can_id = 0x0500 + _module_adress; // 0x05 for master, module id 0xC = 12
    _can_module_id = 0x0700 + _module_adress; // 0x07 for slave, module id 0xC = 12
    _can_error_id = 0x300 + _module_adress; // 0x03 for warning/error, module id 0xC = 12
    _can_socket_id = "can0"; // name within linux ifconfig


    if(!_can_driver.init(_can_socket_id, false)) // read own messages: false
    {
        ROS_ERROR("Error in can socket initialization");
        exit(-2);
    }

    _respListener = _can_driver.createMsgListener(can::MsgHeader(_can_module_id), can::CommInterface::FrameDelegate(this, &Egl90_can_node::handleFrame_response));
    _errorListener = _can_driver.createMsgListener(can::MsgHeader(_can_error_id), can::CommInterface::FrameDelegate(this, &Egl90_can_node::handleFrame_error));

     ROS_INFO("Can socket binding was successful!");
     acknowledge();
     //updateState();

    _timer = _nh.createTimer(ros::Duration(1.0/50.0), &Egl90_can_node::timer_cb, this);

}

void Egl90_can_node::handleFrame_response(const can::Frame &f)
{
    ROS_INFO("Received msg");
    std::map<CMD, STATUS_CMD>::iterator search = _cmd_map.find((CMD)f.data[1]);
    if(search != _cmd_map.end())
    {
        ROS_INFO("Found %d %d", search->first, search->second);
        std::cout << "Found " << search->first << " " << search->second << '\n';
        if (f.dlc >= 2 && f.data[2] == CMD_ERROR)
        {
            search->second = ERROR;
        }
        else
            {
            switch (search->first)
            {
            case CMD_ACK:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {
                    search->second = OK;
                }
                break;
            case CMD_REFERENCE:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {
                    search->second = RUNNING;
                }
                if (f.dlc >= 2 && f.data[2] == CMD_MOVE_BLOCKED)
                {
                    search->second = ERROR;
                }
                if (f.dlc >= 2 && f.data[2] == CMD_POS_REACHED)
                {
                    search->second = OK;
                }
                break;
            case MOVE_VEL:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {
                    search->second = RUNNING;
                }
                if (f.dlc >= 2 && f.data[2] == CMD_MOVE_BLOCKED)
                {
                    search->second = OK;
                }
                break;
            case MOVE_POS:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {
                    search->second = RUNNING;
                }
                if (f.dlc >= 2 && f.data[2] == CMD_MOVE_BLOCKED)
                {
                    search->second = ERROR;
                }
                if (f.dlc >= 2 && f.data[2] == CMD_POS_REACHED)
                {
                    search->second = OK;
                }
                break;
            case CMD_STOP:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {
                    search->second = OK;
                }
                break;
            }
        }
    }
    else {
        ROS_ERROR("Command not found");
    }
    // TODO!!!
}

void Egl90_can_node::handleFrame_error(const can::Frame &f)
{
    ROS_ERROR("Received error msg");
    // TODO!!!
}


void Egl90_can_node::timer_cb(const ros::TimerEvent&)
{
    publishState();
}

bool Egl90_can_node::moveToReferencePos(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ROS_INFO("Sending reference message");
    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 0x01;
    txframe.data[1] = CMD_REFERENCE; //CMD Byte
    txframe.dlc = 2;

//    _can_driver.send(can::toframe("50C#0192"));
    _can_driver.send(txframe);
    _cmd_map[CMD_REFERENCE] = RUNNING;

    do
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
    while (!isDone(CMD_REFERENCE) || _shutdownSignal);

    return true;
}

void Egl90_can_node::acknowledge()
{
    ROS_INFO("Sending acknowledge message");
    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 1; //DLEN
    txframe.data[1] = CMD_ACK; //CMD Byte
    txframe.dlc = 2;


    _cmd_map[CMD_ACK] = RUNNING;
    //_can_driver.send(can::toframe("50C#018B"));
    _can_driver.send(txframe);

}

bool Egl90_can_node::acknowledge(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    acknowledge();
    return true;
}

bool Egl90_can_node::stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{

    ROS_INFO("Sending stop message");
    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 1; //DLEN
    txframe.data[1] = CMD_STOP; //CMD Byte
    txframe.dlc = 2;


    _cmd_map[CMD_STOP] = RUNNING;
//    _can_driver.send(can::toframe("50C#0191"));
    _can_driver.send(txframe);

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
    ROS_INFO("Sending getState message");
    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 1; //DLEN
    txframe.data[1] = GET_STATE; //CMD Byte
    txframe.dlc = 2;

    _cmd_map[GET_STATE] = RUNNING;

//    _can_driver.send(can::toframe("50C#0195"));
    _can_driver.send(txframe);

//    struct can_frame txframe, rxframe1, rxframe2, rxframe3;

//    txframe.can_id = _can_id;
//    txframe.can_dlc = 0x02;
//    txframe.data[0] = 0x01;
//    txframe.data[1] = 0x95;//CMD Byte

//    write(_can_socket, &txframe, sizeof(struct can_frame));

//    ros::Duration(0.01).sleep();
//    // TODO the fragmented CAN message protocol is weird, this will only work if the can is empty besided this module
//    read(_can_socket, &rxframe1, sizeof(struct can_frame));
//    read(_can_socket, &rxframe2, sizeof(struct can_frame));
//    read(_can_socket, &rxframe3, sizeof(struct can_frame));

//    statusData status;
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
//    status.status.position *= 0.001;
//    status.status.speed *= 0.001;
//    _status = status;
    return _status;
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

    ROS_INFO("Sending movePos message");
    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 5; //DLEN
    txframe.data[1] = MOVE_POS; //CMD Byte
    txframe.data[2] = pos.c[0];
    txframe.data[3] = pos.c[1];
    txframe.data[4] = pos.c[2];
    txframe.data[5] = pos.c[3];
    txframe.dlc = 6;

    _cmd_map[MOVE_POS] = RUNNING;

    _can_driver.send(txframe);

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

     vel.f = req.speed;
     cur.f = req.current;

     ROS_INFO("Sending move_vel message");
     can::Frame txframe1 = can::Frame(can::MsgHeader(_can_id));
     can::Frame txframe2 = can::Frame(can::MsgHeader(_can_id));

     txframe1.data[0] = 9; // DLEN Total Data length to come (1Byte CMD + 4Byte Vel + 4Byte Cur)
     txframe1.data[1] = FRAG_START; // First fragment (Fragmentsmarker do not count in DLEN)
     txframe1.data[2] = MOVE_VEL; // Move_vel
     txframe1.data[3] = vel.c[0];
     txframe1.data[4] = vel.c[1];
     txframe1.data[5] = vel.c[2];
     txframe1.data[6] = vel.c[3];
     txframe1.data[7] = cur.c[0];
     txframe1.dlc = 8;

     txframe2.data[0] = 3;
     txframe2.data[1] = FRAG_END; //Last fragment
     txframe2.data[2] = cur.c[1];
     txframe2.data[3] = cur.c[2];
     txframe2.data[4] = cur.c[3];
     txframe2.dlc = 5;

     _cmd_map[MOVE_VEL] = RUNNING;

     _can_driver.send(txframe1);
     _can_driver.send(txframe2);
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

bool Egl90_can_node::isDone(CMD cmd)
{
    if (_cmd_map.count(cmd) == 1)
    {
        switch (_cmd_map[cmd])
        {
            case ERROR:
                ROS_ERROR("COMAND responded with an error");
            case OK:
                return true;
                break;
        }

    }
    else
    {
        ROS_ERROR("Waiting for an answer of a command, which cannot be found!");
    }
    return false;
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

