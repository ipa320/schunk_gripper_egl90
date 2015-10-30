#ifndef EGL90_CAN_H_
#define EGL90_CAN_H_

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

#include <ros/ros.h>

#include "std_srvs/Trigger.h"
#include "ipa325_egl90_can/MovePos.h"
#include "ipa325_egl90_can/MoveGrip.h"

#include <boost/atomic.hpp>

#include <map>

//Variables for converting float Data
union fdata{
    char c[4];
    float f;
};

//Variable for converting status data
struct egl_status{
    float position;
    float speed;
    float current;
    char statusBits;
    char errorCode;
}__attribute__((packed));

union statusData{
    char c[14];
    egl_status status;
};

class Egl90_can_node
{
    enum CMD
    {
        CMD_REFERENCE = 0x92,
        MOVE_POS = 0xB0,
        MOVE_VEL = 0xB5,
        MOVE_GRIP = 0xB7,
        CMD_STOP = 0x91,
        CMD_INFO = 0x8A,
        CMD_ACK = 0x8B,
        CMD_MOVE_BLOCKED = 0x93,
        CMD_POS_REACHED = 0x94,
        CMD_WARNING = 0x89,
        CMD_ERROR = 0x88,
        GET_STATE = 0x95,
        FRAG_ACK = 0x87,
        FRAG_START = 0x84,
        FRAG_MIDDLE = 0x85,
        FRAG_END = 0x86,
        REPLY_OK_1 = 0x4F,
        REPLY_OK_2 = 0x4B
    };

    enum STATUS_CMD
    {
        PENDING,
        RUNNING,
        OK,
        ERROR
    };

public:
    Egl90_can_node();
    void spin();

    statusData updateState();
private:

    static bool _shutdownSignal;
    ros::NodeHandle _nh;

    //ros::Publisher _pub_diagnostics;
    ros::Publisher _pub_joint_states;
    ros::ServiceServer _srv_reference;
    ros::ServiceServer _srv_ack;
    ros::ServiceServer _srv_movePos;
    ros::ServiceServer _srv_moveGrip;
    ros::ServiceServer _srv_stop;
    ros::ServiceServer _srv_getState;

    can::ThreadedSocketCANInterface _can_driver;
    can::CommInterface::FrameListener::Ptr _respListener;
    can::CommInterface::FrameListener::Ptr _errorListener;

    unsigned int _module_adress;
    unsigned int _can_id;
    unsigned int _can_module_id;
    unsigned int _can_error_id;
    std::string _can_socket_id;

    std::map<CMD, STATUS_CMD> _cmd_map;

    ros::Timer _timer;
    statusData _status;
    /**
     * Callback for receiving signal. When SIGINT was received shutdown everything.
     * @param signal
     */
    static void signalHandler(int signal);

    bool moveToReferencePos(std_srvs::Trigger::Request  &req,
                            std_srvs::Trigger::Response &res);

    bool acknowledge(std_srvs::Trigger::Request  &req,
                            std_srvs::Trigger::Response &res);

    bool stop(std_srvs::Trigger::Request  &req,
                            std_srvs::Trigger::Response &res);

    bool getState(std_srvs::Trigger::Request  &req,
                            std_srvs::Trigger::Response &res);

    bool movePos(ipa325_egl90_can::MovePos::Request  &req,
                            ipa325_egl90_can::MovePos::Response &res);

    bool moveGrip(ipa325_egl90_can::MoveGrip::Request  &req,
                            ipa325_egl90_can::MoveGrip::Response &res);

    bool isDone(CMD cmd, bool& error_flag);

    bool publishState();
    void timer_cb(const ros::TimerEvent &);

    void handleFrame_response(const can::Frame &f);
    void handleFrame_error(const can::Frame &f);

};

#endif
