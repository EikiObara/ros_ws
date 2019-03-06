//2018/01/24
//auther : eiki obara

#include <ros/ros.h>

#include <iostream>
#include <cstring>

#include "Definitions.h"
#include "common_functions.h"
#include "mobile_steering.h"

namespace Trl{

const double gain = 0.10;

Steering::Steering() : angle(0){
	if(epos.OpenPort(0) == false){
		ROS_ERROR("epos cannot connection");
	}
	
	epos.InitDevice(EPOS_NUMBER);

	Calibrate();	//起動時の車輪位置をゼロ点にする。

	epos.SetOperationMode(EPOS_NUMBER,Epos2::OperationMode::PROFILE_POSITION);

	//configration for position mode
	epos.SetPositionProfileParam(EPOS_NUMBER,EPOS_PROFILE_VEL,
		EPOS_PROFILE_ACCEL,EPOS_PROFILE_DECEL);
}

Steering::~Steering(){
	epos.DisableDevice(EPOS_NUMBER);
	epos.ClosePort();
}

void Steering::operator()(){
	Run();
}

void Steering::SetAngle(int a){
	angle = a;
}

void Steering::Calibrate(void){
	epos.SetOperationMode(EPOS_NUMBER,Epos2::OperationMode::HOMING);
	epos.SetHomePosition(EPOS_NUMBER);
}

void Steering::Run(){
	int curQc = 0;

	epos.ReadEncoderVal(EPOS_NUMBER,&curQc);

	int tarQc = DegToQuadCount(angle,EPOS_RESOLUTION,EPOS_REDUCTION_RATIO);

	int rpm = (int)(gain * (tarQc - curQc));

	//std::cout << rpm << std::endl;

	epos.SetOperationMode(EPOS_NUMBER,Epos2::OperationMode::VELOCITY);

	epos.DriveMotorVM(EPOS_NUMBER, rpm);
}

}	//namespace Trl
