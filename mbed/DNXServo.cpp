#include "DNXServo.h"

/* ******************************** PUBLIC METHODS ************************************** */

DNXServo::DNXServo(mbed::Serial* portIn, const int& baudIn, const int ReturnLvlIn /*=1*/):
	port(portIn), baud(baudIn), ReturnLvl(ReturnLvlIn){
	// For some reason result is not currently double
	bitPeriod = ((double)1000000.0)/double(baud);
	//bitPeriod = 1000000.0/baud;

	// Set the baud rate of the port
	port.baud(baud);
}

DNXServo::~DNXServo(){}

// SetID
int DNXServo::SetID(const int& ID, const int& newID){
    return dataPush(ID, DNXSERVO_ID, newID);
};

// Read Value from Control Table
int DNXServo::GetValue(const int& ID, const int& address){
	return dataPull(ID, address);
}







/* ******************************** END OF PUBLIC METHODS ************************************** */

/* ******************************** PROTECTED METHODS ************************************** */


// Clear input buffer
void DNXServo::flush() {	
	while (port->readable()) {
		port->getc();
	}
}


// Write buffer to servo 
void DNXServo::write(unsigned char* buf, const int& n) {
	for (int i=0; i < n; i++) {
		port->putc(buf[i]);
	}

	int i=0;
	while(i<n){
		if (port->readable()){	
			port->getc();			//empty buffer because tx has written to rx	(only in case of tx and rx connected)																
			i++;					//rate of the loop does not equal rate of communication
		}
	}
}


// Read reply returns payload length, 0 if error.
int DNXServo::read(unsigned char* buf, const int& nMax /* =255 */) {			//check readBytesUntil()
	int n = 0; 		 	// Bytes read
	int timeout = 0; 	// Timeout

	while ((timeout < 16) && (n < nMax)) {
		if (port->readable()) {
			buf[n] = port->getc();
			n++;
			timeout = 0;
		}
		else{
			wait_ms(bitPeriod);																	
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
		pc.print_debug("CCW out of range\n");
		return 1024;	
	} 

	else if (result<0){
		pc.print_debug("CW out of range\n");
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
	if(!pc.get_debug()) return;
	pc.print_debug("PACKET {");
	for (int i=0; i < bytes; i++) {
		pc.print_debug(to_hex(buf[i])+" ");
	}
	pc.print_debug(" } \n");
}



/* ******************************** END OF PROTECTED METHODS ************************************** */
