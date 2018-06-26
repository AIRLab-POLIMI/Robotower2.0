#include<ros/ros.h>
#include "SerialPort.h"


arduino_serial::SerialPort::SerialPort(char const *portName):portname(portName){
	this->baud = B57600;
	this->connected = false;
	arduino_serial::SerialPort::connect();
}

arduino_serial::SerialPort::SerialPort(char const *portName, speed_t baud):portname(portName), baud(baud){
	this->connected = false;
	arduino_serial::SerialPort::connect();
}

void arduino_serial::SerialPort::connect(){
	ROS_INFO_STREAM("Openning connection with " << portname);
	/* Open the file descriptor in non-blocking mode */
	this->handler = open(portname, O_RDWR | O_NOCTTY);

	/* Set up the control structure */
	struct termios toptions;

	/* Get currently set options for the tty */
	tcgetattr(this->handler, &toptions);

	/* Set custom options */

	/* 9600 baud */
	cfsetispeed(&toptions, this->baud);

	/* 8 bits, no parity, no stop bits */
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	/* no hardware flow control */
	toptions.c_cflag &= ~CRTSCTS;
	/* enable receiver, ignore status lines */
	toptions.c_cflag |= CREAD | CLOCAL;
	/* disable input/output flow control, disable restart chars */
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
	/* disable canonical input, disable echo,
	 disable visually erase chars,
	 disable terminal-generated signals */
	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	/* disable output processing */
	toptions.c_oflag &= ~OPOST;

	/* wait for 12 characters to come in before read returns */
	/* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
	/* CHARACTERS HAVE COME IN! */
	toptions.c_cc[VMIN] = 12;
	/* no minimum time to wait before read returns */
	toptions.c_cc[VTIME] = 0;

	/* commit the options */
	tcsetattr(this->handler, TCSANOW, &toptions);

	/* Wait for the Arduino to reset */
	usleep(ARDUINO_WAIT_TIME);
	this->connected = true;
}

arduino_serial::SerialPort::~SerialPort(){
	close(this->handler);
}


int arduino_serial::SerialPort::read_port(char *buffer, unsigned int buf_size){
   /* Flush anything already in the serial buffer */
   tcflush(this->handler, TCIFLUSH);
   /* read up to 128 bytes from the handler */
   int n = read(this->handler, buffer, buf_size);
   return n;
}


bool arduino_serial::SerialPort::write_port(char *buffer, unsigned int buf_size){
	return false;
}

bool arduino_serial::SerialPort::isConnected(){
	return this->connected;
}

int arduino_serial::SerialPort::read_port_until(std::stringstream & buffer, const char* delimiter, int timeout){
	char byte[1];  // read expects an array, so we give it a 1-byte array
	int read_status;
	
    do { 
        read_status = read(this->handler, byte, 1);  // read a char at a time
        if (read_status == -1) return -1;     // couldn't read
        if (read_status == 0) {
            usleep(1*1000);  // wait 1 msec try again
            timeout--;
            if(timeout==0) return -2;
            continue;
        }
		buffer << byte[0];					// store byte into the string stream.
    } while(byte[0] != delimiter[0] && timeout > 0);

    return read_status;
}