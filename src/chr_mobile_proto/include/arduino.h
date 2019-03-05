#ifndef __ARDUINO_IF_H__
#define __ARDUINO_IF_H__

#include <termios.h>
#include <string>

class Arduino {
 public:
    Arduino();
    ~Arduino();

    void Write(const std::string wrd);
    std::string Read();

 private:
    int fd;
    struct termios oldtio;
    struct termios newtio;

    bool Init();
    void Term();

};

#endif  //__ARDUINO_IF_H__
