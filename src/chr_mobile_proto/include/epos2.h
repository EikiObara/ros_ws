#ifndef __EPOS2_H__
#define __EPOS2_H__

#include <string>
//#include "Definitions.h"

#define MAX_FOLLOWING_ERROR 2000 //[quad count]
#define MAX_PROFILE_VELOCITY 4000 //[rpm]
#define MAX_ACCELERATION 4000 //[rpm/s]

#define PROFILE_VELOCITY 600
#define PROFILE_ACCELERATION 240
#define PROFILE_DECELERATION 240

#define HOMING_ACCELERATION 50
#define SPEED_SWITCH 50
#define SPEED_INDEX 10
#define CURRENT_THRESHOLD 500
#define HOME_OFFSET 0
#define HOME_POSITION 0

class Epos2 {
public:
    //use for VCS_MoveToPosition()
    enum BaseType {
        ABSOLUTE = 1,
        RELATIVE = 0
    };

    //use for VCS_MoveToPosition()
    enum Timing {
        IMMEDIATRY = 1,
        WAIT = 0
    };
    
    //use for VCS_SetOperationMode()
    enum OperationMode {
        PROFILE_POSITION = OMD_PROFILE_POSITION_MODE,
        PRODILE_VELOCITY = OMD_PROFILE_VELOCITY_MODE,
        HOMING = OMD_HOMING_MODE,
        INTERPOLATED_POSITION = OMD_INTERPOLATED_POSITION_MODE,
        POSITION = OMD_POSITION_MODE,
        VELOCITY = OMD_VELOCITY_MODE,
        CURRENT = OMD_CURRENT_MODE,
        MASTER_ENCODER = OMD_MASTER_ENCODER_MODE,
        STEP_DIRECTION = OMD_STEP_DIRECTION_MODE
    };

    Epos2();
    ~Epos2();

    //port open and close
    bool OpenPort(const unsigned short portNum);
    bool ClosePort();

    //device initialize
    void InitDevice(const unsigned short nodeId);

    //state control
    void EnableDevice(const unsigned short nodeId);
    void DisableDevice(const unsigned short nodeId);
    void ClearFault(const unsigned short nodeId);
    
    //parameter setting
    void SetLimitParam(const unsigned short nodeId, const unsigned int maxFollowingError = MAX_FOLLOWING_ERROR,
                         const unsigned int maxProfileVel = MAX_PROFILE_VELOCITY,
                         const unsigned int maxAccel = MAX_ACCELERATION);
    void SetPositionProfileParam(const unsigned short nodeId, const unsigned int profileVel = PROFILE_VELOCITY,
                                    const unsigned int profileAccel = PROFILE_ACCELERATION,
                                    const unsigned int profileDecel = PROFILE_DECELERATION);
    void SetVelocityProfileParam(const unsigned short nodeId,
                                    const unsigned int profileAccel = PROFILE_ACCELERATION,
                                    const unsigned int profileDecel = PROFILE_DECELERATION);
    void SetHomingModeParam(const unsigned short nodeId,
                            const unsigned int homingAccel = HOMING_ACCELERATION,
                            const unsigned int speedSwitch = SPEED_SWITCH,
                            const unsigned int speedIndex = SPEED_INDEX,
                            const unsigned int currentThreshold = CURRENT_THRESHOLD,
                            const long homeOffset = HOME_OFFSET, const long homePosition = HOME_POSITION);
    void SetOperationMode(const unsigned short nodeId, const OperationMode mode);
    void GetOperationMode(const unsigned short nodeId, char* currentMode);
    void SetHomePosition(const unsigned short nodeId);
    void SetNegativeHomePosition(const unsigned short nodeId);
    void SetPositiveHomePosition(const unsigned short nodeId);

    //operation
    void ReadEncoderVal(const unsigned short nodeId, int* currentQc);
    void DriveMotorPPM(const unsigned short nodeId, const long targetQc,
                        const BaseType baseType, const Timing timing);
    void WaitForTargetReached(const unsigned short nodeId, const unsigned int timeout/*[ms]*/);
    void DriveMotorPM(const unsigned short nodeId, const long targetQc);
    void DriveMotorVM(const unsigned short nodeId, const long targetVelRpm);

    void HaltPM(const unsigned short nodeId);
    void HaltVM(const unsigned short nodeId);

    void SetCurrent(const unsigned short nodeId, const short currentMust);
    void GetCurrent(const unsigned short nodeId, short *currentIs);

private:
    void* keyHandle;
    unsigned int errorCode;

    bool OpenDevice(const std::string deviceName, const std::string protocolStackName,
                    const std::string interfaceName, const std::string portName, const int baudrate);

    void LogError(const unsigned short nodeId, const std::string functionName);
    void DisplayDeviceError(const unsigned short nodeId);
};

#endif //EPOS2_H_
