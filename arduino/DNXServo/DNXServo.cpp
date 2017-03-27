#include "DNXServo.h"

/* ******************************** PUBLIC METHODS ************************************** */

DNXServo::DNXServo(HardwareSerial& portIn, const long int& baudIn, const int& DebugLvlIn /*=1*/):
	port(&portIn), baud(baudIn), DebugLvl(DebugLvlIn) {

	bitPeriod = 1000000.0/baud;
	
	if(port==&Serial1) port_num=1;
	else if(port==&Serial2) port_num=2;
	else if(port==&Serial3) port_num=3;
	else Serial.println("Unknown port");
	
	port->begin(baud);

	if(DebugLvl){
		Serial.print("Serial");
		Serial.print(port_num);
		Serial.print(" started at ");
		Serial.print(baud);
		Serial.print(" baud rate and ");	
		Serial.print(bitPeriod);
		Serial.println(" bit period");
	}
}

DNXServo::~DNXServo(){}

unsigned char DNXServo::getPort() const{
	return port_num;
}

// Enable Printing Debugging Messages on Serial
void DNXServo::EnableDebugging(){
	DebugLvl = 1;
}

// Disable Printing Debugging Messages on Serial
void DNXServo::DisableDebugging(){
	DebugLvl = 0;
}

// SetID
int DNXServo::SetID(const int& ID, const int& newID){
    return dataPush(ID, DNXSERVO_ID, newID);
};

// Read Value from Control Table
int DNXServo::readValue(const int& ID, const int& address){
	return dataPull(ID, address);
}







/* ******************************** END OF PUBLIC METHODS ************************************** */

/* ******************************** PROTECTED METHODS ************************************** */


// Clear input buffer
void DNXServo::flush() {	
	while (port->available()) {
		port->read();
	}
}


// Write buffer to servo 
void DNXServo::write(unsigned char* buf, const int& n) {
	for (int i=0; i < n; i++) {
		port->write(buf[i]);
	}

	int i=0;
	while(i<n){
		if (port->available()){	
			port->read();			//empty buffer because tx has written to rx	(only in case of tx and rx connected)																
			i++;					//rate of the loop does not equal rate of communication
		}
	}
}


// Read reply returns payload length, 0 if error.
int DNXServo::read(const int& ID, unsigned char* buf, const int& nMax /* =255 */) {			//check readBytesUntil()

	int n = 0; 		 	// Bytes read
	int timeout = 0; 	// Timeout

	while ((timeout < 16) && (n < nMax)) {		// Timeout 2bytes max
		if (port->available()) {
			buf[n] = port->read();
			n++;
			timeout = 0;
		}
		else{
			delayMicroseconds(bitPeriod);																	
			timeout++;		
		}															
	}

	return n;
}


// CW - positive input, CCW - negative input
int DNXServo::angleScale(const double& angle){
	
	// 2.61799387799 rad = 150 degrees
	int result = 512 - ((angle/2.61799387799)*512);
	// 0 is end CW and 1024 is end CCW

	if (result>1024){
		if(DebugLvl) Serial.println("CCW out of range");
		return 1024;	
	} 

	else if (result<0){
		if(DebugLvl) Serial.println("CW out of range");
		return 0;	
	}  
	
	else return result;
}


// Length of address
int DNXServo::AddressLenght(const int& address, const unsigned char * TWO_BYTE_ADDRESSES) {
	bool found=false;
	
	for(int i=0; i<11 && !found; i++){
		if (address == TWO_BYTE_ADDRESSES[i]) found=true;
	}
	
	if(found) return 2;
	else return 1;
}


// packetPrint
void DNXServo::packetPrint(const int& bytes, unsigned char* buf) {
	Serial.print("PACKET {");
	for (int i=0; i < bytes; i++) {
		Serial.print(buf[i], HEX);
		Serial.print(" ");
	}
	Serial.println(" } ");
}



/* ******************************** END OF PROTECTED METHODS ************************************** */
