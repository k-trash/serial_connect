#include "serial_connect/serial_connect.hpp"

#include <iostream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/signal.h>

SerialConnect::SerialConnect(void){
	device_name = "/dev/ttyACM0";
	device_num = -1;
	connection = false;
	read_success = 0;
	baud_rate = B115200;
	error_out = true;
}

void SerialConnect::setSerial(std::string device_name_, speed_t baud_rate_, bool error_out_){
	if(connection){
		closeSerial();
	}

	device_name = device_name_;
	device_num = -1;
	connection = false;
	read_success = 0;
	baud_rate = baud_rate_;
	error_out = error_out_;
}

void SerialConnect::openSerial(void){
	struct termios conf_tio;

	device_num = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	fcntl(device_num, F_SETFL, 0);
	
	tcgetattr(device_num, &conf_tio);

	cfmakeraw(&conf_tio);	

	cfsetispeed(&conf_tio, baud_rate);
	cfsetospeed(&conf_tio, baud_rate);

	conf_tio.c_cc[VMIN] = 0;
	conf_tio.c_cc[VTIME] = 0;

	tcsetattr(device_num, TCSANOW, &conf_tio);
	if(device_num >= 0){
		connection = true;
		infoSerial("serial open");
	}else{
		connection = false;
		errorSerial("couldn't open");
	}
}

void SerialConnect::closeSerial(void){
	close(device_num);
	connection = false;
	infoSerial("serial close");
}

void SerialConnect::setInterrupt(void (*call_back_)(int)){
	struct sigaction saio;

	call_back = call_back_;
	saio.sa_handler = call_back_;
	saio.sa_flags = 0;
	saio.sa_restorer = NULL;

	sigaction(SIGIO, &saio, NULL);

	fcntl(device_num, F_SETFL, FNDELAY);
	fcntl(device_num, F_SETOWN, getpid());
	fcntl(device_num, F_SETFL, O_ASYNC);

	infoSerial("interrupt set up");
}

void SerialConnect::writeSerial(const uint8_t *write_data_, size_t data_size_){
	int ret_write = 0;
	if(!connection){
		reconnectSerial();
	}
	if(connection){
		ret_write = write(device_num, write_data_, data_size_);
		if(ret_write < 0){
			connection = false;
			closeSerial();
			errorSerial("couldn't write to");
		}
	}
}

int SerialConnect::readSerial(void){
	int ret_read = 0;
	if(connection){
		ret_read = read(device_num, recv_data, sizeof(recv_data));
		if(ret_read > 0){
			read_success++;
			return ret_read;
		}else{
			errorSerial("couldn't read from");
		}
	}else{
		reconnectSerial();
	}
	return -1;
}

bool SerialConnect::isConnect(void){
	return connection;
}

bool SerialConnect::isSerial(void){
	if(read_success > 0){
		read_success = 0;
		return true;
	}else{
		return false;
	}
}

void SerialConnect::reconnectSerial(void){
	if(!connection){
		openSerial();
		if(connection){
			setInterrupt(call_back);
		}
	}
}

void __attribute__((weak)) SerialConnect::errorSerial(std::string error_str_){
	if(error_out){
		std::cerr << "Serial Fail: " << error_str_ << ' ' << device_name << std::endl;
	}
}

void __attribute__((weak)) SerialConnect::infoSerial(std::string info_str_){
	std::cout << "Serial INFO: " << info_str_ << ' ' << device_name << std::endl;
}