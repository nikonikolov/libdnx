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

	inline void print_debug(const std::string& msg);

	void set_debug();
	bool get_debug();

private:

	mbed::Serial pc_usb;
	bool debug;
};


inline void PCSerial::print_debug(const std::string& msg){
	if(!debug) return;
	pc_usb.printf(msg.c_str());
}


#endif
