#ifndef SERIALPORT_H
#define SERIALPORT_H

#define ARDUINO_WAIT_TIME 1000*1000

#include <stdio.h>
#include <cstdlib>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sstream>      // std::stringstream

namespace arduino_serial{
    class SerialPort{
    private:
        int handler;
        bool connected;
        char const *portname;
        speed_t baud;
        struct termios toptions;

        void connect();

    public:
        SerialPort(char const *portName);
        SerialPort(char const *portName, speed_t baud);
        ~SerialPort();

        int read_port(char *buffer, unsigned int buf_size);
        bool write_port(char *buffer, unsigned int buf_size);
        int read_port_until(std::stringstream &buffer, const char* delimiter, int timeout);
        bool isConnected();
    };
}
#endif // SERIALPORT_H
