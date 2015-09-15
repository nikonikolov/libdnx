#include "DNXServo.h"

DNXServo::DNXServo(HardwareSerial& port, const long int& baud):
	_port(&port), _baud(baud) {

	//_port=&port;
	//_baud = baud;
	_bitPeriod = 1000000.0/_baud;
	
	if(_port==&Serial1) _port_num=1;
	else if(_port==&Serial2) _port_num=2;
	else if(_port==&Serial3) _port_num=3;
	else Serial.println("Unknown port");
	
	_port->begin(_baud);
	
	Serial.print("Serial");
	Serial.print(_port_num);
	Serial.print(" started at ");
	Serial.print(_baud);
	Serial.print(" baud rate and ");	
	Serial.print(_bitPeriod);
	Serial.println(" bit period");
}


unsigned char DNXServo::getPort() const{
	return _port_num;
}


// packetPrint
void DNXServo::packetPrint(int bytes, unsigned char* buf) {
	Serial.print("PACKET {");
	for (int i=0; i < bytes; i++) {
		Serial.print(buf[i], HEX);
		Serial.print(" ");
	}
	Serial.println(" } ");
}


// Clear input buffer
void DNXServo::flush() {	
	while (_port->available()) {
		_port->read();
	}
}


// Length of address
int DNXServo::addrLength(int address, const unsigned char * two_byte) {
	bool found=false;
	
	for(int i=0; i<11 && !found; i++){
		if (address == two_byte[i]) found=true;
	}
	
	if(found) return 2;
	else return 1;
}


// CW - positive input, CCW - negative input
int DNXServo::angleScale(const double& angle){
	
	// 2.61799387799 rad = 150 degrees
	int result = 512 - ((angle/2.61799387799)*512);

	if (result>1024) return 1024;
	else if (result<0) return 0; 
	else return result;
}


// Write buffer to servo 
void DNXServo::write(unsigned char* buf, int n) {
	for (int i=0; i < n; i++) {
		_port->write(buf[i]);
	}

	for (int i=0; i < n; ) {
		if (_port->available()){	
			_port->read();			//empty buffer because tx has written to rx	(only in case of tx and rx connected)																
			i++;					//rate of the loop does not equal rate of communication
		}
	}
}


// Read reply returns payload length, 0 if error.
int DNXServo::read(int ID, unsigned char* buf, int nMax /* =255 */) {			//check readBytesUntil()

	int n = 0; 		 	// Bytes read
	int timeout = 0; 	// Timeout
	//bool flag = true;
	while ((timeout < 500) && (n < nMax)) {
		if (_port->available()) {
			buf[n] = _port->read();
			n++;
			timeout = 0;
		}
																	
		delayMicroseconds(_bitPeriod);																	
		timeout++;
		
		/*
		if ((n > 6) && (flag)) {
			int l = length(buf);
			if (l < nMax) {
				nMax = l;
			}
			flag = false;
		}
		*/
	}

	return n;
}


// SetID
int DNXServo::SetID(int ID, int newID){
    return dataPush(ID, DNXSERVO_ID, newID);
};

// Read Value from Control Table
int DNXServo::GetValue(int ID, int address){
	return dataPull(ID, address);
}