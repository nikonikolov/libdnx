#ifndef PCSERIAL_H
#define PCSERIAL_H

//#include "../mbed/api/mbed.h"
#include "mbed.h"
#include <string>

using std::string;

class PCSerial{

public:

	PCSerial(const bool& debug_in = false);
	~PCSerial();

	void print_debug(const std::string& msg);

	void set_debug();
	bool get_debug();

private:

	mbed::Serial pc_usb;
	bool debug;
};


#endif
