#ifndef EGL90_CAN_H_
#define EGL90_CAN_H_

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

#include <ros/ros.h>

#include "std_srvs/Trigger.h"
#include "ipa325_egl90_can/MovePos.h"
#include "ipa325_egl90_can/MoveGrip.h"

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

#include <map>
#include <utility>

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
        CMD_NOT_FOUND,
        PENDING,
        RUNNING,
        OK,
        ERROR
    };

    enum ERROR_CODE
    {
        INFO_BOOT = 1,
        INFO_NO_RIGHTS = 3,
        INFO_UNKNOWN_COMMAND = 4,
        INFO_FAILED = 5,
        NOT_REFERENCED = 6,
        INFO_SEARCH_SINE_VECTOR = 7,
        INFO_NO_ERROR = 8,
        INFO_COMMUNICATION_ERROR = 9,
        INFO_TIMEOUT = 16,
        INFO_UNKNOWN_AXIS_INDEX = 17,
        INFO_WRONG_BAUDRATE = 22,
        INFO_CHECKSUM = 25,
        INFO_MESSAGE_LENGTH = 29,
        INFO_WRONG_PARAMETER = 30,
        ERROR_TEMP_LOW = 112,
        ERROR_TEMP_HIGH = 113,
        ERROR_LOGIC_LOW = 114,
        ERROR_LOGIC_HIGH = 115,
        ERROR_MOTOR_VOLTAGE_LOW = 116,
        ERROR_MOTOR_VOLTAGE_HIGH = 117,
        ERROR_CABLE_BREAK = 118,
        ERROR_OVERSHOOT = 130,
        ERROR_WRONG_RAMP_TYPE = 200,
        ERROR_CONFIG_MEMORY = 210,
        ERROR_PROGRAM_MEMORY = 211,
        ERROR_INVALIDE_PHRASE = 212,
        ERROR_SOFT_LOW = 213,
        ERROR_SOFT_HIGH = 214,
        ERROR_SERVICE = 216,
        ERROR_FAST_STOP = 217,
        ERROR_TOW = 218,
        ERROR_VPC3 = 219,
        ERROR_FRAGMENTATION = 220,
        ERROR_COMMUTATION = 221,
        ERROR_CURRENT = 222,
        ERROR_I2T = 223,
        ERROR_INITIALIZE = 224,
        ERROR_INTERNAL = 225,
        ERROR_TOO_FAST = 228,
        ERROR_RESOLVER_CHECK_FAILED = 229,
        ERROR_MATH = 236,
    };

public:
    Egl90_can_node();
    void spin();

    void updateState();
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
    ros::ServiceServer _srv_cleanUp;

    can::ThreadedSocketCANInterface _can_driver;
    can::CommInterface::FrameListener::Ptr _respListener;
    can::CommInterface::FrameListener::Ptr _errorListener;

    unsigned int _module_adress;
    unsigned int _can_id;
    unsigned int _can_module_id;
    unsigned int _can_error_id;
    unsigned int _timeout_ms;
    std::string _can_socket_id;

    boost::mutex _mutex;
    boost::mutex _condition_mutex;
    boost::condition_variable _cond;

    std::map<CMD, std::pair<int, STATUS_CMD> > _cmd_map;

    std::map<CMD, std::string> _cmd_str;
    std::map<STATUS_CMD, std::string> _status_cmd_str;
    std::map<ERROR_CODE, std::string> _error_str;

    ros::Timer _timer_update;
    ros::Timer _timer_publish;
    statusData _status;
    statusData _tempStatus;
    boost::mutex _statusMutex;

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

    bool cleanUp(std_srvs::Trigger::Request  &req,
                            std_srvs::Trigger::Response &res);

    bool movePos(ipa325_egl90_can::MovePos::Request  &req,
                            ipa325_egl90_can::MovePos::Response &res);

    bool moveGrip(ipa325_egl90_can::MoveGrip::Request  &req,
                            ipa325_egl90_can::MoveGrip::Response &res);

    bool isDone(CMD cmd, bool& error_flag);

    bool publishState();

    void handleFrame_response(const can::Frame &f);
    void handleFrame_error(const can::Frame &f);

    bool setState(CMD command, STATUS_CMD status);
    bool setState(CMD command, STATUS_CMD status, bool warn);
    STATUS_CMD getState(CMD command);
    bool addState(Egl90_can_node::CMD command);
    bool addState(Egl90_can_node::CMD command, Egl90_can_node::STATUS_CMD status);
    bool removeState(Egl90_can_node::CMD command);

    void fillStrMaps();
};

#endif
