#include <iostream>
#include <cerrno>
#include <cstring>
#include <unistd.h>

#include "arduino.h"
#include "mobile_propulsion.h"

namespace Trl{

Propulsion::Propulsion():speed(0){}

Propulsion::~Propulsion(){}

void Propulsion::operator()(){
	Run();
}

void Propulsion::SetSpeed(int s){
	this->speed = s;
}

bool Propulsion::Run(){
	direction type;
	
	if(speed > 0){
		type = FORE;
	}else if(speed < 0){
		type = BACK;
	}else{
		type = STOP;
	}

	std::string msg;

	msg = std::to_string(type);
	msg += ",";
	msg += std::to_string(speed);
	msg += ",";
	msg += "1";
	msg += ";";
	
	ard.Write(msg);

	return true;
}

}	//namespace Trl
