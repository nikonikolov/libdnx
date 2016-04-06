#include "include.h"

PCSerial pc(false);

string to_hex(const uint8_t& dec){
	stringstream ss;
	ss<<std::hex<<(int)((uint32_t)dec);
	return ss.str();
}


string itos(const int& num){
	stringstream ss;
	ss<<num;
	return ss.str();
}

string dtos(const double& num){
	stringstream ss;
	ss<<num;
	return ss.str();	
}
