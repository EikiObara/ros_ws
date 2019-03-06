//2018/01/24
//auther : eiki obara

#ifndef __MOBILE_PROPULSION_H__
#define __MOBILE_PROPULSION_H__

#include "arduino.h"

namespace Trl{

class Propulsion{
public:
	enum direction{
		STOP,
		FORE,
		BACK
	};

	Propulsion();
	~Propulsion();

	void operator ()();

	void SetSpeed(int s = 0);

	bool Run();

private:
	Arduino ard;
	int speed;
};

}	//namespace Trl

//Arduino側との通信が突然終了するなど、不測の状態になったら、
//Arduino側のプログラムを停止させる。

#endif	//__MOBILE_PROPULSION_H__
