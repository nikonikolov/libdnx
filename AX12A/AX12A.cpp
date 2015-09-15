#include "AX12A.h"
#include "../DNXServo/DNXServo.h"

#include <math.h>

AX12A::AX12A(HardwareSerial& port, long int baud):
	DNXServo(port, baud) {}


const unsigned char AX12A::two_byte[11] = { 0, 6, 8, 14, 30, 32, 34, 36, 38, 40, 48 };

// Length of address
int AX12A::addrLength(int address) {
	return DNXServo::addrLength(address, two_byte);
}

// Dynamixel Communication 1.0 Checksum
unsigned char AX12A::update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    
    unsigned char crc_accum=0;
    
    // Header bytes (0xFF, 0xFF) do not get included in the checksum  
    for(unsigned char i = 2; i < data_blk_size; i++) {
        crc_accum += data_blk_ptr[i];
    }
    
    return ~(crc_accum);
}


// change
int AX12A::statusError(unsigned char* buf, int n) {
	
	// Minimum return length
	if (n < 6) {
		flush();
		Serial.println("READING CORRUPTION");
		return -1; 
	}

	if ((buf[0]!=0xFF)||(buf[1]!=0xFF)) {
		flush();
		Serial.println("WRONG RETURN HEADER");
		packetPrint(n, buf);
		return -1; 
	}

	unsigned char checksum=update_crc(buf, n-1);
	// The last byte does not get included in the checksum
	if(checksum != buf[n-1]){
		flush();
		Serial.println("WRONG RETURN CHECKSUM");
		packetPrint(n, buf);
		Serial.print("CHECKSUM READ IS ");
		Serial.println(checksum, HEX);
		return -1;
	}

	if ( (buf[3]+4) != n ) {
		flush();
		Serial.println("WRONG RETURN LENGTH");
		packetPrint(n, buf);
		return -1;
	}

	if(buf[4]!=0 ){
		Serial.println("STATUS ERROR ");
		// bit 0
			 if ( !(buf[4] & 0x01) ) Serial.println("VOLTAGE OUT OF RANGE");	
		// bit 1
		else if ( !(buf[4] & 0x02) ) Serial.println("REQUIRED POSITION OUT OF RANGE");
		// bit 2
		else if ( !(buf[4] & 0x04) ) Serial.println("TEMPERATURE OUT OF RANGE");
		// bit 3
		else if ( !(buf[4] & 0x08) ) Serial.println("COMMAND OUT OF RANGE");
		// bit 4
		else if ( !(buf[4] & 0x10) ) Serial.println("CORRUPTED PACKAGE SENT - CRC DOES NOT MATCH");
		// bit 5
		else if ( !(buf[4] & 0x20) ) Serial.println("LOAD OUT OF RANGE");
		// bit 6
		else if ( !(buf[4] & 0x40) ) Serial.println("UNDEFINED OR MISSING COMMAND");
		// bit 7
		else if ( !(buf[4] & 0x80) ) Serial.println("GLITCH");
		return -1;
	}

	return 0;
}


// Packs data and sends it to the servo
// Dynamixel Communication 1.0 Protocol: Header, ID, Packet Length, Instruction, Parameter, 16bit CRC
int AX12A::send(int ID, int bytes, unsigned char* parameters, unsigned char ins) {
	
	unsigned char buf[bytes+6]; // Packet

	// Header
	buf[0] = 0xFF;
	buf[1] = 0xFF;

	// ID
	buf[2] = ID;

	// Packet Length
	buf[3] = bytes+2;

	// Instruction
	buf[4] = ins;

	// Parameter
	for (int i=0; i < bytes; i++) {
		buf[5+i] = parameters[i];
	}

	// Checksum
	buf[bytes+5] = update_crc(buf, bytes+5);

	// Transmit
	write(buf, bytes+6);
	
	if (ID == ID_Broadcast){
		return 0;	
	}
	
	// Read reply
	Serial.println("- Reading");
	int n = read(ID, reply_buf);
	if (n == 0) {
		Serial.println("Could not read status packet. Zero bytes read");
		return 0;
	}

	Serial.print("- Read ");
	Serial.print(n);
	Serial.println(" bytes");

	return statusError(reply_buf, n); // Error code
}


// dataPack sets the parameters in char array and returns length.
int AX12A::dataPack(unsigned char ins, unsigned char ** parameters, int address, int value /*=0*/){

	unsigned char* data; 
	
	int adrl = addrLength(address);

	int size;
	if (ins == AX_INS_Write) size = adrl+1;
	else size = 2;

	data = new unsigned char[size];
	data[0] = LOBYTE(address);
	
	if (ins == AX_INS_Read){
		
		data[1] = LOBYTE(adrl);	
	}

	if (ins == AX_INS_Write){

		data[adrl] = HIBYTE(value);
		data[1] = LOBYTE(value);			// if adrl is 1, data[2] will be overwritten and again the correct packet will be sent
	}

	*parameters = data;

	return size;
}


// dataPush is a generic wrapper for single value SET instructions for public methods
int AX12A::dataPush(int ID, int address, int value){
	flush(); // Flush reply for safety															
	
	unsigned char* parameters;
    int bytes = dataPack(AX_INS_Write, &parameters, address, value);

    int ec = send(ID, bytes, parameters, AX_INS_Write);
   	
   	delete[] parameters;

   	return ec;
}


// dataPull is a generic wrapper for single value GET instructions for public methods
int AX12A::dataPull(int ID, int address){
	flush(); // Flush reply	for safety														
	
	unsigned char* parameters;
    int bytes = dataPack(AX_INS_Read, &parameters, address);

    int size = parameters[1];
   	
   	int ec = send(ID, bytes, parameters, AX_INS_Read);

   	delete[] parameters;
   	
   	if (ec != 0) {
   		return ec;
   	}

	//packetPrint(15, buf);
	if ( ((unsigned char)ID) == reply_buf[2] ){
   		if (size==2) return (unsigned int)MAKEWORD(reply_buf[5], reply_buf[6]);
   		else return (unsigned int)reply_buf[5];	
   	}

   	else{
   		Serial.println("WRONG ID REPLIED");
   		return -1;
   	}
}


// SetBaud
// 1: 1Mbps, 3: 500 000, 4: 400 000, 7: 250 000, 9: 200 000, 16: 115200, 34: 57600, 103: 19200, 207: 9600
int AX12A::SetBaud(int ID, int rate) {
	if ( rate != 1 && rate != 3 && rate != 4 && rate != 7 && rate != 9 && rate != 16  && rate != 34 && rate != 103 && rate != 207 ) {
		Serial.println("Incorrect baud rate");
		return 1;
	}

	return dataPush(ID, AX_BAUD_RATE, rate);
}


// Set which commands return status
// 0: None, 1: Read, 2: All.
int AX12A::SetReturnLevel(int ID, int lvl) {
	return dataPush(ID, AX_RETURN_LEVEL, lvl);
}


// Turn LED on (0x01) and off (0x00)
int AX12A::SetLED(int ID, int value){
	return dataPush(ID, AX_LED, value);
}


// SetGoalPosition
// 1024 = -150 degrees CCW, 512 = 0 degrees (ORIGIN), 0 = +150 degrees CW
int AX12A::SetGoalPosition(int ID, int angle){
	return dataPush(ID, AX_GOAL_POSITION, angle);
}

int AX12A::SetGoalPosition(int ID, double angle){
	return dataPush(ID, AX_GOAL_POSITION, angleScale(angle));
}



int AX12A::SetGoalVelocity(int ID, int velocity){
	return dataPush(ID, AX_GOAL_VELOCITY, velocity);
}


int AX12A::SetGoalTorque(int ID, int torque){
	return dataPush(ID, AX_MAX_TORQUE, torque);
}


int AX12A::SetPunch(int ID, int punch){
	return dataPush(ID, AX_PUNCH, punch);
}
