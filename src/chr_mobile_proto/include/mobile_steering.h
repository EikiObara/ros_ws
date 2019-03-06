//2018/01/24
//auther : eiki obara

#ifndef __MOBILE_STEERING_H__
#define __MOBILE_STEERING_H__

#include "epos2.h"

namespace Trl{

const unsigned short EPOS_NUMBER = 10;

//profile position mode config value
const unsigned short EPOS_PROFILE_ACCEL = 100;
const unsigned short EPOS_PROFILE_DECEL = 100;
const unsigned short EPOS_PROFILE_VEL = 300;

//DegToQuadCount constant number
//Please dont change if you use the moter.
const unsigned short EPOS_RESOLUTION = 512;
const unsigned short EPOS_REDUCTION_RATIO = 100;

class Steering{
public:
	Steering();
	~Steering();

	void operator()();

	void SetAngle(int a);

	void Run();

private:
	Epos2 epos;
	void Calibrate(void);
	int angle;
};

}	//namespace Trl

#endif //__MOBILE_STEERING_H__
