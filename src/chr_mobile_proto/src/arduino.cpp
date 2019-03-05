#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstring>
#include <cerrno>

#include "arduino.h"

#include <string>

#define BAUDRATE B9600
#define ARDUINO_SERIAL_PORT "/dev/ttyACM"

Arduino::Arduino() : fd(0) {
    Init();
}

Arduino::~Arduino(){
    Term();
}

bool Arduino::Init() {
	for(int i = 0; i < 10; ++i){
		std::string name;
		
		name = ARDUINO_SERIAL_PORT;
		name += std::to_string(i);

		this->fd = open(name.c_str(), O_RDWR | O_NOCTTY);

		if(i > 9){
			std::perror("arduino initialize");
			std::exit(EXIT_FAILURE);
		}

		//ポートを開けない場合falseをreturn
		if(this->fd <= 0){
			std::cout << name << " port is nothing" << std::endl;
			continue;
		}else{
			std::cout << name << " port open " << std::endl;
			break;
		}
	}

	tcgetattr(this->fd, &oldtio);
	newtio = oldtio;
	cfsetspeed(&newtio, BAUDRATE);
	tcflush(this->fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);
	
	return true;
}

void Arduino::Term(){
    tcsetattr(fd, TCSANOW, &oldtio);
    close(this->fd);
}

void Arduino::Write(const std::string wrd){
	if(write(this->fd, wrd.c_str(), wrd.length()) < 0){
		std::cout << "[Arduino] write error" << std::endl;
	}
}

std::string Arduino::Read(){
    int flg = 0;
    char buff[255] = {""};
    
    flg = read(this->fd,buff,255);

//    std::cout << "flg = " << flg << std::endl;

    if(flg == 0){
	    return std::string(buff);
    }

    return std::string(buff);
}

