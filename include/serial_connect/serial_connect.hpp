#pragma once

#include <string>
#include <termios.h>

class SerialConnect{
	public:
		uint8_t recv_data[256];

		SerialConnect(void);
		void setSerial(std::string device_name_, speed_t baud_rate_, bool error_out_);
		void openSerial(void);
		void closeSerial(void);
		void setInterrupt(void (*call_back_)(int));
		void writeSerial(const uint8_t *write_data_, size_t data_size_);
		int readSerial(void);
		bool isConnect(void);
		bool isSerial(void);
		void reconnectSerial(void);
		void errorSerial(std::string error_str_);
		void infoSerial(std::string info_str_);
	private:
		void (*call_back)(int);
		bool error_out;
		bool connection;
		int device_num;
		int read_success;
		speed_t baud_rate;
		std::string device_name;
};
