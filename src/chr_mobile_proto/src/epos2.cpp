#include "Definitions.h"
#include "epos2.h"

#include <string.h> //strcpy()
#include <iostream> //cout,cerr,hex, right
#include <iomanip>  //setw()
#include <string>   //string

Epos2::Epos2(){
    keyHandle = NULL;
    errorCode = 0;
}

Epos2::~Epos2(){}

bool Epos2::OpenPort(const unsigned short portNum){
    std::string deviceName("EPOS2");
    std::string protocolStackName("MAXON SERIAL V2");
    //std::string protocolStackName("CANopen");
    std::string interfaceName("USB");
    //std::string interfaceName("CAN_ixx_usb 0");
    std::string portName("USB" + std::to_string(portNum));
    //std::string portName("CAN" + std::to_string(portNum));
    int baudrate = 1000000;

    if(OpenDevice(deviceName, protocolStackName, interfaceName, portName, baudrate)){
        return true;
    }

    return false;
}

bool Epos2::ClosePort(){
    if(VCS_CloseDevice(keyHandle, &errorCode) == 0){
        LogError(0, "VCS_CloseDevice");
        return false;
    }

    //if(VCS_CloseAllDevices(&errorCode) == 0){
    //    LogError(0, "VCS_closeAllDevices");
    //    return false;
    //}

    return true;
}

void Epos2::InitDevice(const unsigned short nodeId){
    ClearFault(nodeId);
    EnableDevice(nodeId);
    SetLimitParam(nodeId);
    SetPositionProfileParam(nodeId);
    SetHomingModeParam(nodeId);
}

void Epos2::EnableDevice(const unsigned short nodeId){

    int enabled=0;
    if(VCS_GetEnableState(keyHandle, nodeId, &enabled, &errorCode)==0){
        LogError(nodeId, "VCS_GetEnableState");
    }

    if(!enabled){
        if(VCS_SetEnableState(keyHandle, nodeId, &errorCode)==0){
            LogError(nodeId, "VCS_SetEnableState");
        }
    }
}

void Epos2::DisableDevice(const unsigned short nodeId){
    int disable=0;
    if(VCS_GetDisableState(keyHandle, nodeId, &disable, &errorCode)==0){
        LogError(nodeId, "VCS_GetDisableState");
    }

    if(!disable){
        if(VCS_SetDisableState(keyHandle, nodeId, &errorCode)==0){
            LogError(nodeId, "VCS_SetDisableState");
        }
    }
}

void Epos2::ClearFault(const unsigned short nodeId){
    int fault = 0;

    if(VCS_GetFaultState(keyHandle, nodeId, &fault, &errorCode)==0){
        LogError(nodeId, "VCS_GetFaultState");
    }

    if(fault){
        DisplayDeviceError(nodeId);

        if(VCS_ClearFault(keyHandle, nodeId, &errorCode)==0){
            LogError(nodeId, "VCS_ClearFault");
        } else {
            std::cout << "Epos2[node" << nodeId << "]";
            std::cout << " clear fault" << std::endl;
        }
    }
}

void Epos2::SetLimitParam(const unsigned short nodeId, const unsigned int maxFollowingError,
                            const unsigned int maxProfileVel, const unsigned int maxAccel){

    if(VCS_SetMaxFollowingError(keyHandle, nodeId, maxFollowingError, &errorCode)==0){
        LogError(nodeId, "VCS_SetMaxFollowingError");
    }
    if(VCS_SetMaxProfileVelocity(keyHandle, nodeId, maxProfileVel, &errorCode)==0){
        LogError(nodeId, "VCS_SetMaxProfileVelocity");
    }
    if(VCS_SetMaxAcceleration(keyHandle, nodeId, maxAccel, &errorCode)==0){
        LogError(nodeId, "VCS_SetMaxAcceleration");
    }
}

void Epos2::SetPositionProfileParam(const unsigned short nodeId, const unsigned int profileVel,
                                    const unsigned int profileAccel, const unsigned int profileDecel){

    if(VCS_SetPositionProfile(keyHandle, nodeId, profileVel, profileAccel, profileDecel, &errorCode)==0){
        LogError(nodeId, "VCS_SetPositionProfile");
    }
}

void Epos2::SetVelocityProfileParam(const unsigned short nodeId, const unsigned int profileAccel,
                                    const unsigned int profileDecel){

    if(VCS_SetVelocityProfile(keyHandle, nodeId, profileAccel, profileDecel, &errorCode)==0){
        LogError(nodeId, "VCS_SetVelocityProfile");
    }
}

void Epos2::SetHomingModeParam(const unsigned short nodeId, const unsigned int homingAccel,
                                const unsigned int speedSwitch, const unsigned int speedIndex,
                                const unsigned int currentThreshold, const long homeOffset,
                                const long homePosition){

    if(VCS_SetHomingParameter(keyHandle, nodeId, homingAccel, speedSwitch, speedIndex, homeOffset, currentThreshold, homePosition, &errorCode)==0){
        LogError(nodeId, "VCS_SetHomingParameter");
    }
}

void Epos2::SetOperationMode(const unsigned short nodeId, const OperationMode mode){

    if(VCS_SetOperationMode(keyHandle, nodeId, static_cast<char>(mode), &errorCode)==0){
        LogError(nodeId, "VCS_SetOperationMode");
    }
}

void Epos2::GetOperationMode(const unsigned short nodeId, char* operationMode){
    if(VCS_GetOperationMode(keyHandle, nodeId, operationMode, &errorCode)==0){
        LogError(nodeId, "VCS_GetOperationMode");
    }
}

void Epos2::SetHomePosition(const unsigned short nodeId){
    SetOperationMode(nodeId, OperationMode::HOMING);
    SetHomingModeParam(nodeId);
    if(VCS_FindHome(keyHandle, nodeId, HM_ACTUAL_POSITION, &errorCode)==0){
        LogError(nodeId, "VCS_FindHome");
    }
}

void Epos2::SetNegativeHomePosition(const unsigned short nodeId){
	SetOperationMode(nodeId,OperationMode::HOMING);
	SetHomingModeParam(nodeId);
	if(VCS_FindHome(keyHandle,nodeId,HM_CURRENT_THRESHOLD_NEGATIVE_SPEED,&errorCode)==0){
		LogError(nodeId,"VCS_FindHome:current Threshold negative speed");
	}
}

void Epos2::SetPositiveHomePosition(const unsigned short nodeId){
	SetOperationMode(nodeId,OperationMode::HOMING);
	SetHomingModeParam(nodeId);
	if(VCS_FindHome(keyHandle,nodeId,HM_CURRENT_THRESHOLD_POSITIVE_SPEED,&errorCode)==0){
		LogError(nodeId,"VCS_FindHome:current Threshold positive speed");
	}
}

void Epos2::ReadEncoderVal(const unsigned short nodeId, int* currentQc){
    if(VCS_GetPositionIs(keyHandle, nodeId, currentQc, &errorCode)==0){
        LogError(nodeId, "VCS_GetPositionIs");
    }
}

void Epos2::DriveMotorPPM(const unsigned short nodeId, const long targetQc,
                            const BaseType baseType, const Timing timing){
    if(VCS_MoveToPosition(keyHandle, nodeId, targetQc, (int)baseType, (int)timing, &errorCode)==0){
        LogError(nodeId, "VCS_MoveToPosition");
    }
}

void Epos2::WaitForTargetReached(const unsigned short nodeId, const unsigned int timeout/*[ms]*/){
    if(VCS_WaitForTargetReached(keyHandle, nodeId, timeout, &errorCode)==0){
        LogError(nodeId, "WaitForTargetReached");
    }
}

void Epos2::DriveMotorPM(const unsigned short nodeId, const long targetQc){
    if(VCS_SetPositionMust(keyHandle, nodeId, targetQc, &errorCode)==0){
        LogError(nodeId, "VCS_SetPositionMust");
    }
}

void Epos2::DriveMotorVM(const unsigned short nodeId, const long targetVelRpm){
    if(VCS_SetVelocityMust(keyHandle, nodeId, targetVelRpm, &errorCode)==0){
        LogError(nodeId, "VCS_SetVelocityMust"); 
    }
}

void Epos2::HaltPM(const unsigned short nodeId){
    if(VCS_HaltPositionMovement(keyHandle, nodeId, &errorCode)==0){
        LogError(nodeId, "VCS_HaltPositionMovement"); 
    }
}

void Epos2::HaltVM(const unsigned short nodeId){
    if(VCS_HaltVelocityMovement(keyHandle, nodeId, &errorCode)==0){
        LogError(nodeId, "VCS_HaltVelocityMovement"); 
    }
}

void Epos2::SetCurrent(const unsigned short nodeId, const short currentMust/*[mA]*/){
    if(VCS_SetCurrentMust(keyHandle, nodeId, currentMust, &errorCode)==0){
        LogError(nodeId, "VCS_SetCurrentMust");
    }
}

void Epos2::GetCurrent(const unsigned short nodeId, short *currentIs){
    VCS_GetCurrentIs(keyHandle, nodeId, currentIs, &errorCode);
}

//open port and create handle
bool Epos2::OpenDevice(const std::string deviceName, const std::string protocolStackName,
                        const std::string interfaceName, const std::string portName, const int baudrate){

    char pPortName[255] = {};
    char pDeviceName[255] = {};
    char pProtocolStackName[255] = {};
    char pInterfaceName[255] = {};

    strcpy(pDeviceName, deviceName.c_str());
    strcpy(pProtocolStackName, protocolStackName.c_str());
    strcpy(pInterfaceName, interfaceName.c_str());
    strcpy(pPortName, portName.c_str());
    
    keyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &errorCode);

    if(keyHandle != 0 && errorCode == 0){
        unsigned int baudrateBuff = 0;
        unsigned int timeout = 0;

        if(VCS_GetProtocolStackSettings(keyHandle, &baudrateBuff, &timeout, &errorCode)==0){
            LogError(0, "VCS_GetProtocolStackSettings");
            return false;
        } else {
            if(VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, &errorCode)==0){
                LogError(0, "VCS_SetProtocolStackSettings");
                return false;
            } else {
                if(VCS_GetProtocolStackSettings(keyHandle, &baudrateBuff, &timeout, &errorCode)==0){
                    LogError(0, "VCS_GetProtocolStackSettings");
                    return false;
                } else {
                    if(baudrate == (int)baudrateBuff){
                        return true;
                    } else {
                        LogError(0, "VCS_OpenDevice");
                        return false;
                    }
                }
            }
        }
    } else {
        LogError(0, "VCS_OpenDevice");
        return false;
    }
}

void Epos2::LogError(const unsigned short nodeId, const std::string functionName){
    if(nodeId > 0){
        std::cerr << "Epos2[node" << nodeId << "] ";
    }

    std::cerr << functionName << "() failed ";

    if(nodeId > 0){
        std::cerr << "(errorCode=0x" << std::hex << errorCode << ")" << std::dec;
    }

    std::cerr << std::endl;
}

void Epos2::DisplayDeviceError(const unsigned short nodeId){
    unsigned char numOfDeviceError = 0;
    unsigned int deviceErrorCode = 0;

    //デバイスエラーの個数を取得
    if(VCS_GetNbOfDeviceError(keyHandle, nodeId, &numOfDeviceError, &errorCode) == 0){
        LogError(nodeId, "VCS_GetNbOfDeviceError");
    } else {
        for(unsigned char errorNum = 1; errorNum <= numOfDeviceError; errorNum++){
            //エラーコードを取得して表示
            if(VCS_GetDeviceErrorCode(keyHandle, nodeId, errorNum, &deviceErrorCode, &errorCode) == 0){
                break;
            } else {
                std::cout << "Epos2[node" << nodeId << "]";
                std::cout << " device error occurred (errorCode=0x" << std::hex << deviceErrorCode << ")" << std::dec << std::endl;
            }
        }
    }
}

