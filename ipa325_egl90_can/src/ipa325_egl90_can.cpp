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

    fillStrMaps();
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
    _timeout_ms = 5000; //5s


    if(!_can_driver.init(_can_socket_id, false)) // read own messages: false
    {
        ROS_ERROR("Error in can socket initialization");
        exit(-2);
    }

    _respListener = _can_driver.createMsgListener(can::MsgHeader(_can_module_id), can::CommInterface::FrameDelegate(this, &Egl90_can_node::handleFrame_response));
    _errorListener = _can_driver.createMsgListener(can::MsgHeader(_can_error_id), can::CommInterface::FrameDelegate(this, &Egl90_can_node::handleFrame_error));

     ROS_INFO("Can socket binding was successful!");
     std_srvs::Trigger::Request  req;
     std_srvs::Trigger::Response res;
     acknowledge(req, res);
     //updateState();

    _timer = _nh.createTimer(ros::Duration(1.0/50.0), &Egl90_can_node::timer_cb, this);

}

void Egl90_can_node::handleFrame_response(const can::Frame &f)
{
    ROS_INFO("Received msg CMD=%x, %s", f.data[1], _cmd_str[(CMD)f.data[1]].c_str());
    std::map<CMD, std::pair<int, STATUS_CMD> >::iterator search = _cmd_map.find((CMD)f.data[1]);

    if(search == _cmd_map.end()) // CMD not found in list, probably a spontanious msg
    {
        switch(f.data[1])
        {
        case CMD_INFO:
            break;
        case CMD_MOVE_BLOCKED:
            setState(CMD_REFERENCE, ERROR);
            setState(MOVE_POS, ERROR);
            setState(MOVE_VEL, OK);

            break;
        case CMD_WARNING: //TODO
        case CMD_POS_REACHED:
            setState(CMD_REFERENCE, OK);
            setState(MOVE_POS, OK);
            setState(MOVE_VEL, OK);

            break;
        case CMD_ERROR:
            setState(CMD_REFERENCE, ERROR);
            setState(MOVE_POS, ERROR);
            setState(MOVE_VEL, ERROR);

            break;
        case FRAG_START:
            if (f.data[2] == GET_STATE)
            {
                setState(GET_STATE, RUNNING);
                _tempStatus.c[0] = f.data[3];
                _tempStatus.c[1] = f.data[4];
                _tempStatus.c[2] = f.data[5];
                _tempStatus.c[3] = f.data[6];
                _tempStatus.c[4] = f.data[7];
            }
            break;
        case FRAG_MIDDLE:
            if (getState(GET_STATE) == RUNNING)
            {
                _tempStatus.c[5] = f.data[2];
                _tempStatus.c[6] = f.data[3];
                _tempStatus.c[7] = f.data[4];
                _tempStatus.c[8] = f.data[5];
                _tempStatus.c[9] = f.data[6];
                _tempStatus.c[10] = f.data[7];
            }
            break;
        case FRAG_END:
            if (getState(GET_STATE) == RUNNING)
            {
                _tempStatus.c[11] = f.data[2];
                _tempStatus.c[12] = f.data[3];
                _tempStatus.c[13] = f.data[4];

                _tempStatus.status.position *= 0.001;
                _tempStatus.status.speed *= 0.001;

                boost::mutex::scoped_lock lock(_statusMutex);
                _status = _tempStatus;
                lock.unlock();

                setState(GET_STATE, OK);
                _cond.notify_one();
            }
            break;
        }

    }
    else
    {
        ROS_INFO("Got response %x %x, %s %s", search->first, search->second.second, _cmd_str[(CMD)search->first].c_str(), _cmd_str[(CMD)search->second.second].c_str());
        if (f.dlc >= 2 && f.data[2] == CMD_ERROR)
        {
            search->second.second = ERROR;
        }
        else
            {
            switch (search->first)
            {
            case CMD_ACK:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {
                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = OK;
                    lock.unlock();
                    _cond.notify_one();
                }
                break;
            case CMD_REFERENCE:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {
                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = RUNNING;
                    lock.unlock();
                    _cond.notify_one();
                }
                if (f.dlc >= 2 && f.data[2] == CMD_MOVE_BLOCKED)
                {
                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = ERROR;
                    lock.unlock();
                    _cond.notify_one();
                }
                if (f.dlc >= 2 && f.data[2] == CMD_POS_REACHED)
                {

                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = OK;
                    lock.unlock();
                    _cond.notify_one();
                }
                break;
            case MOVE_VEL:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {

                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = RUNNING;
                    lock.unlock();
                    _cond.notify_one();
                }
                if (f.dlc >= 2 && f.data[2] == CMD_MOVE_BLOCKED)
                {

                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = OK;
                    lock.unlock();
                    _cond.notify_one();
                }
                break;
            case MOVE_POS:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {
                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = RUNNING;
                    lock.unlock();
                    _cond.notify_one();
                }
                if (f.dlc >= 2 && f.data[2] == CMD_MOVE_BLOCKED)
                {

                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = ERROR;
                    lock.unlock();
                    _cond.notify_one();
                }
                if (f.dlc >= 2 && f.data[2] == CMD_POS_REACHED)
                {
                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = OK;
                    lock.unlock();
                    _cond.notify_one();
                }
                break;
            case CMD_STOP:
                if (f.dlc >= 3 && f.data[2] == REPLY_OK_1 && f.data[3] == REPLY_OK_2)
                {

                    boost::mutex::scoped_lock lock(_mutex);
                    search->second.second = OK;
                    lock.unlock();
                    _cond.notify_one();

                    // Special case that if stop was called, other motion commands are canceled!
                    setState(CMD_REFERENCE, ERROR);
                    setState(MOVE_POS, ERROR);
                    setState(MOVE_VEL, OK);

                }
                break;
            }
        }
    }
}

void Egl90_can_node::handleFrame_error(const can::Frame &f)
{
    ROS_ERROR("Received error msg: %x %x, %s %s", f.data[1], f.data[2], _error_str[(ERROR_CODE)f.data[1]].c_str(), _error_str[(ERROR_CODE)f.data[2]].c_str());
    // TODO!!!
    switch(f.data[1])
    {
    case CMD_ERROR:
        setState(CMD_REFERENCE, ERROR, false);

        setState(MOVE_POS, ERROR, false);

        // Check for softstop and may put MOVE_VEL to ok
        switch(f.data[2])
        {
            case ERROR_SOFT_LOW:
            case ERROR_SOFT_HIGH:
                setState(MOVE_VEL, OK, false);
                break;
            default:
                setState(MOVE_VEL, ERROR, false);
                break;
        }
        break;
    }

    ROS_WARN("For now just try acknoledging it!");
    std_srvs::Trigger::Request  req;
    std_srvs::Trigger::Response res;
    acknowledge(req, res);
    ros::Duration(0.5).sleep();
}

bool Egl90_can_node::setState(Egl90_can_node::CMD command, Egl90_can_node::STATUS_CMD status)
{
    return setState(command, status, true);
}

bool Egl90_can_node::setState(Egl90_can_node::CMD command, Egl90_can_node::STATUS_CMD status, bool createIfNotFound)
{
        if (getState(command) == CMD_NOT_FOUND)
        {
            if (createIfNotFound)
            {
                boost::mutex::scoped_lock lock(_mutex);
                // new creation
                _cmd_map[command] = std::make_pair(1, status);
                lock.unlock();
                _cond.notify_one();
                return true;
            }
            else
            {
                return false;
            }
        }
        else // CMD was found
        {
                boost::mutex::scoped_lock lock(_mutex);
                //instance counter++
                _cmd_map[command].first++;
                lock.unlock();
                _cond.notify_one();
                return true;

        }
}

Egl90_can_node::STATUS_CMD Egl90_can_node::getState(Egl90_can_node::CMD command)
{
    STATUS_CMD status = CMD_NOT_FOUND;

    boost::mutex::scoped_lock lock(_mutex);
    std::map<CMD, std::pair<int, STATUS_CMD> >::iterator search = _cmd_map.find(command);
    if (search != _cmd_map.end())
    {
        status = search->second.second;
    }
    lock.unlock();
    return status;
}

void Egl90_can_node::fillStrMaps()
{
    _cmd_str[CMD_REFERENCE] = "CMD_REFERENCE";
    _cmd_str[MOVE_POS] = "MOVE_POS";
    _cmd_str[MOVE_VEL] = "MOVE_VEL";
    _cmd_str[MOVE_GRIP] = "MOVE_GRIP";
    _cmd_str[CMD_STOP] = "CMD_STOP";
    _cmd_str[CMD_INFO] = "CMD_INFO";
    _cmd_str[CMD_ACK] = "CMD_ACK";
    _cmd_str[CMD_MOVE_BLOCKED] = "CMD_MOVE_BLOCKED";
    _cmd_str[CMD_POS_REACHED] = "CMD_POS_REACHED";
    _cmd_str[CMD_WARNING] = "CMD_WARNING";
    _cmd_str[CMD_ERROR] = "CMD_ERROR";
    _cmd_str[GET_STATE] = "GET_STATE";
    _cmd_str[FRAG_ACK] = "FRAG_ACK";
    _cmd_str[FRAG_START] = "FRAG_START";
    _cmd_str[FRAG_MIDDLE] = "FRAG_MIDDLE";
    _cmd_str[FRAG_END] = "FRAG_END";
    _cmd_str[REPLY_OK_1] = "REPLY_OK_1";
    _cmd_str[REPLY_OK_2] = "REPLY_OK_2";

    _status_cmd_str[CMD_NOT_FOUND] = "CMD_NOT_FOUND";
    _status_cmd_str[PENDING] = "PENDING";
    _status_cmd_str[RUNNING] = "RUNNING";
    _status_cmd_str[OK] = "OK";
    _status_cmd_str[ERROR] = "ERROR";

    _error_str[INFO_BOOT] = "INFO_BOOT";
    _error_str[INFO_NO_RIGHTS] = "INFO_NO_RIGHTS";
    _error_str[INFO_UNKNOWN_COMMAND] = "INFO_UNKNOWN_COMMAND";
    _error_str[INFO_FAILED] = "INFO_FAILED";
    _error_str[NOT_REFERENCED] = "NOT_REFERENCED";
    _error_str[INFO_SEARCH_SINE_VECTOR] = "INFO_SEARCH_SINE_VECTOR";
    _error_str[INFO_NO_ERROR] = "INFO_NO_ERROR";
    _error_str[INFO_COMMUNICATION_ERROR] = "INFO_COMMUNICATION_ERROR";
    _error_str[INFO_TIMEOUT] = "INFO_TIMEOUT";
    _error_str[INFO_UNKNOWN_AXIS_INDEX] = "INFO_UNKNOWN_AXIS_INDEX";
    _error_str[INFO_WRONG_BAUDRATE] = "INFO_WRONG_BAUDRATE";
    _error_str[INFO_CHECKSUM] = "INFO_CHECKSUM";
    _error_str[INFO_MESSAGE_LENGTH] = "INFO_MESSAGE_LENGTH";
    _error_str[INFO_WRONG_PARAMETER] = "INFO_WRONG_PARAMETER";
    _error_str[ERROR_TEMP_LOW] = "ERROR_TEMP_LOW";
    _error_str[ERROR_TEMP_HIGH] = "ERROR_TEMP_HIGH";
    _error_str[ERROR_LOGIC_LOW] = "ERROR_LOGIC_LOW";
    _error_str[ERROR_LOGIC_HIGH] = "ERROR_LOGIC_HIGH";
    _error_str[ERROR_MOTOR_VOLTAGE_LOW] = "ERROR_MOTOR_VOLTAGE_LOW";
    _error_str[ERROR_MOTOR_VOLTAGE_HIGH] = "ERROR_MOTOR_VOLTAGE_HIGH";
    _error_str[ERROR_CABLE_BREAK] = "ERROR_CABLE_BREAK";
    _error_str[ERROR_OVERSHOOT] = "ERROR_OVERSHOOT";
    _error_str[ERROR_WRONG_RAMP_TYPE] = "ERROR_WRONG_RAMP_TYPE";
    _error_str[ERROR_CONFIG_MEMORY] = "ERROR_CONFIG_MEMORY";
    _error_str[ERROR_PROGRAM_MEMORY] = "ERROR_PROGRAM_MEMORY";
    _error_str[ERROR_INVALIDE_PHRASE] = "ERROR_INVALIDE_PHRASE";
    _error_str[ERROR_SOFT_LOW] = "ERROR_SOFTLOW";
    _error_str[ERROR_SOFT_HIGH] = "ERROR_SOFT_HIGH";
    _error_str[ERROR_SERVICE] = "ERROR_SERVICE";
    _error_str[ERROR_FAST_STOP] = "ERROR_FAST_STOP";
    _error_str[ERROR_TOW] = "ERROR_TOW";
    _error_str[ERROR_VPC3] = "ERROR_VPC3";
    _error_str[ERROR_FRAGMENTATION] = "ERROR_FRAGMENTATION";
    _error_str[ERROR_COMMUTATION] = "ERROR_COMMUTATION";
    _error_str[ERROR_CURRENT] = "ERROR_CURRENT";
    _error_str[ERROR_I2T] = "ERROR_I2T";
    _error_str[ERROR_INITIALIZE] = "ERROR_INITIALIZE";
    _error_str[ERROR_INTERNAL] = "ERROR_INTERNAL";
    _error_str[ERROR_TOO_FAST] = "ERROR_TOO_FAST";
    _error_str[ERROR_RESOLVER_CHECK_FAILED] = "ERROR_RESOLVER_CHECK_FAILED";
    _error_str[ERROR_MATH] = "ERROR_MATH";

}

void Egl90_can_node::timer_cb(const ros::TimerEvent&)
{
    publishState();
}

bool Egl90_can_node::moveToReferencePos(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 0x01;
    txframe.data[1] = CMD_REFERENCE; //CMD Byte
    txframe.dlc = 2;
    bool error_flag = false;

//    _can_driver.send(can::toframe("50C#0192"));
    setState(CMD_REFERENCE, PENDING);

    unsigned int counterMs = 0;
    do
    {
        ROS_INFO("Sending reference message");
        _can_driver.send(txframe);
        do
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
            counterMs++;
        }
        while (!_shutdownSignal && !isDone(CMD_REFERENCE, error_flag) && counterMs <= _timeout_ms);
    }
    while (counterMs >= _timeout_ms);

    if (error_flag)
    {
        res.success = false;
        res.message = "Module did reply with error!";
    }
    else
    {
        res.success = true;
        res.message = "Module did reply properly!";
    }
    return true;
}


bool Egl90_can_node::acknowledge(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 1; //DLEN
    txframe.data[1] = CMD_ACK; //CMD Byte
    txframe.dlc = 2;

    setState(CMD_ACK, PENDING);

    //_can_driver.send(can::toframe("50C#018B"));

    bool error_flag = false;

    unsigned int counterMs = 0;
    do
    {
        ROS_INFO("Sending acknowledge message");
        _can_driver.send(txframe);
        do
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
            counterMs++;
        }
        while (!_shutdownSignal && !isDone(CMD_ACK, error_flag) && counterMs <= _timeout_ms);
    }
    while (counterMs >= _timeout_ms);

    if (error_flag)
    {
        res.success = false;
        res.message = "Module did reply with error!";
    }
    else
    {
        res.success = true;
        res.message = "Module did reply properly!";
    }
    return true;
}

bool Egl90_can_node::stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{

    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 1; //DLEN
    txframe.data[1] = CMD_STOP; //CMD Byte
    txframe.dlc = 2;

    bool error_flag = false;

    setState(CMD_STOP, PENDING);

//    _can_driver.send(can::toframe("50C#0191"));
    unsigned int counterMs = 0;
    do
    {
        ROS_INFO("Sending stop message");
        _can_driver.send(txframe);
        do
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
            counterMs++;
        }
        while (!_shutdownSignal && !isDone(CMD_STOP, error_flag) && counterMs <= _timeout_ms);
    }
    while (counterMs >= _timeout_ms);

    if (error_flag)
    {
        res.success = false;
        res.message = "Module did reply with error!";
    }
    else
    {
        res.success = true;
        res.message = "Module did reply properly!";
    }
    return true;
}

void Egl90_can_node::updateState()
{
    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 1; //DLEN
    txframe.data[1] = GET_STATE; //CMD Byte
    txframe.dlc = 2;

    setState(GET_STATE, PENDING);

//    _can_driver.send(can::toframe("50C#0195"));
    ROS_INFO("Sending getState message");
    _can_driver.send(txframe);
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

    boost::mutex::scoped_lock lock(_statusMutex);
    js.position.push_back(_status.status.position);
    js.velocity.push_back(_status.status.speed);
    js.effort.push_back(_status.status.current);
    lock.unlock();
    _pub_joint_states.publish(js);
}

bool Egl90_can_node::movePos(ipa325_egl90_can::MovePos::Request &req, ipa325_egl90_can::MovePos::Response &res)
{
    fdata pos;
    pos.f = req.position;

    can::Frame txframe = can::Frame(can::MsgHeader(_can_id));
    txframe.data[0] = 5; //DLEN
    txframe.data[1] = MOVE_POS; //CMD Byte
    txframe.data[2] = pos.c[0];
    txframe.data[3] = pos.c[1];
    txframe.data[4] = pos.c[2];
    txframe.data[5] = pos.c[3];
    txframe.dlc = 6;

    bool error_flag = false;
    setState(MOVE_POS, PENDING);


    unsigned int counterMs = 0;
    do
    {
        ROS_INFO("Sending move_pos message");
        _can_driver.send(txframe);
        do
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
            counterMs++;
        }
        while (!_shutdownSignal && !isDone(MOVE_POS, error_flag) && counterMs <= _timeout_ms);
    }
    while (counterMs >= _timeout_ms);

    if (error_flag)
    {
        res.success = false;
        res.message = "Module did reply with error!";
    }
    else
    {
        res.success = true;
        res.message = "Module did reply properly!";
    }
    return true;
}

bool Egl90_can_node::moveGrip(ipa325_egl90_can::MoveGrip::Request &req, ipa325_egl90_can::MoveGrip::Response &res)
{
     ROS_INFO("move_grip seems not to be available in module 12, this is the alternativ implementation using velocity cmd and underlying current control!");
     ROS_WARN("The parameter you give in this command will be the default parameters for future move_pos commands!");

     fdata vel, cur;

     vel.f = req.speed;
     cur.f = req.current;

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

     bool error_flag = false;

     setState(MOVE_VEL, PENDING);

     unsigned int counterMs = 0;
     do
     {
         ROS_INFO("Sending move_vel message");
         _can_driver.send(txframe1);
         _can_driver.send(txframe2);
         do
         {
             ros::Duration(0.01).sleep();
             ros::spinOnce();
             counterMs++;
         }
         while (!_shutdownSignal && !isDone(MOVE_VEL, error_flag) && counterMs <= _timeout_ms);
     }
     while (counterMs >= _timeout_ms);

     if (error_flag)
     {
         res.success = false;
         res.message = "Module did reply with error!";
     }
     else
     {
         res.success = true;
         res.message = "Module did reply properly!";
     }
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

bool Egl90_can_node::isDone(CMD cmd, bool& error_flag)
{
    bool isDone = false;
    if (_cmd_map.count(cmd) == 1)
    {
        switch (_cmd_map[cmd].second)
        {
            case ERROR:
                ROS_ERROR("COMMAND %s responded with an error", _cmd_str[cmd].c_str());
                error_flag = true;
                //TODO
            case OK:
                ROS_INFO("Command %s is done with ok", _cmd_str[cmd].c_str());
                boost::mutex::scoped_lock lock(_mutex);
                if (_cmd_map[cmd].first <= 1)
                {
                    _cmd_map.erase(cmd);
                }
                else
                {
                    _cmd_map[cmd].first--;
                }
                lock.unlock();
                isDone = true;
            break;
        }
    }
    else
    {
        ROS_ERROR("Waiting for an answer of a command, which cannot be found! %x, %s", cmd, _cmd_str[cmd].c_str());
    }
    return isDone;
}

void Egl90_can_node::spin()
{
    ros::Rate rate(100);

    //wait for shutdown
    while(!_shutdownSignal && ros::ok())
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

