#ifndef INCLUDE_H
#define INCLUDE_H

#include <iostream>
#include <string>
#include <sstream>
#include "mbed.h"
#include "PCSerial.h"

using std::string;
using std::stringstream;

extern PCSerial pc;
extern mbed::Serial deviceAX12A;
extern mbed::Serial deviceXL320;

string to_hex(const int& dec);
string itos(const int& num);

#endif
