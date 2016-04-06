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

string to_hex(const uint8_t& dec);
string itos(const int& num);
string dtos(const double& num);

#endif
