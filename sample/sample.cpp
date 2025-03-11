#include <serial_connect/serial_connect.hpp>

#include <iostream>

int main(int argc, char *argv[]){
	SerialConnect serial;
	uint8_t write_data[] = "Hello World!";

	serial.setSerial("/dev/ttyUSB0", B115200, true);
	serial.openSerial();

	serial.writeSerial(write_data, sizeof(write_data)/sizeof(uint8_t));
	int ret = serial.readSerial();

	if(ret > 0){
		std::cout << serial.recv_data << std::endl; 
	}

	serial.closeSerial();

	return 0;
}