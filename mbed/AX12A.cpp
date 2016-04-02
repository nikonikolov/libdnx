#include "AX12A.h"


/* ******************************** PUBLIC METHODS ************************************** */

AX12A::AX12A(mbed::Serial* portIn, const int& baudIn, const int ReturnLvlIn /*=1*/):
	DNXServo(portIn, baudIn, ReturnLvlIn){
		
	SetReturnLevel(ID_Broadcast, ReturnLvl);
	pc.print_debug("AX12A object attached to serial at baud rate " + itos(baudIn) + " and bitPeriod of " + itos(bitPeriod) + "\n");
}

AX12A::~AX12A(){}



// 1: 1Mbps, 3: 500 000, 4: 400 000, 7: 250 000, 9: 200 000, 16: 115200, 34: 57600, 103: 19200, 207: 9600
int AX12A::SetBaud(const int& ID, const int& rate) {
	if ( rate != 1 && rate != 3 && rate != 4 && rate != 7 && rate != 9 && rate != 16  && rate != 34 && rate != 103 && rate != 207 ) {
		pc.print_debug("Incorrect baud rate\n");
		return 1;
	}

	return dataPush(ID, AX_BAUD_RATE, rate);
}

// Set which commands return status; 0: None, 1: Read, 2: All.
int AX12A::SetReturnLevel(const int& ID, const int& lvl) {
	return dataPush(ID, AX_RETURN_LEVEL, lvl);
}



// 1024 = -150 degrees CCW, 512 = 0 degrees (ORIGIN), 0 = +150 degrees CW
int AX12A::SetGoalPosition(const int& ID, const int& angle){
	return dataPush(ID, AX_GOAL_POSITION, angle);
}

int AX12A::SetGoalPosition(const int& ID, const double& angle){
	return dataPush(ID, AX_GOAL_POSITION, angleScale(angle));
}

int AX12A::SetGoalVelocity(const int& ID, const int& velocity){
	return dataPush(ID, AX_GOAL_VELOCITY, velocity);
}

int AX12A::SetGoalTorque(const int& ID, const int& torque){
	return dataPush(ID, AX_MAX_TORQUE, torque);
}

int AX12A::SetPunch(const int& ID, const int& punch){
	return dataPush(ID, AX_PUNCH, punch);
}

// Turn LED on (0x01) and off (0x00)
int AX12A::SetLED(const int& ID, const int& value){
	return dataPush(ID, AX_LED, value);
}

/* ******************************** PUBLIC METHODS END ************************************** */

/* ******************************** PRIVATE METHODS ************************************** */


// Dynamixel Communication 1.0 Checksum
unsigned char AX12A::update_crc(unsigned char *data_blk_ptr, const unsigned short& data_blk_size) {
    
    unsigned char crc_accum=0;
    
    // Header bytes (0xFF, 0xFF) do not get included in the checksum  
    for(unsigned char i = 2; i < data_blk_size; i++) {
        crc_accum += data_blk_ptr[i];
    }
    
    return ~(crc_accum);
}


// Return Length of Address
int AX12A::AddressLenght(const int& address) {
	return DNXServo::AddressLenght(address, TWO_BYTE_ADDRESSES);
}


int AX12A::statusError(unsigned char* buf, const int& n) {
	
	// Minimum return length
	if (n < 6) {
		flush();
		pc.print_debug("READING CORRUPTION\n");
		return -1; 
	}

	if ((buf[0]!=0xFF)||(buf[1]!=0xFF)) {
		flush();
		pc.print_debug("WRONG RETURN HEADER\n");
		packetPrint(n, buf);
		return -1; 
	}

	unsigned char checksum=update_crc(buf, n-1);
	// The last byte does not get included in the checksum
	if(checksum != buf[n-1]){
		flush();
			pc.print_debug("WRONG RETURN CHECKSUM\n");
			packetPrint(n, buf);
			pc.print_debug("CHECKSUM READ IS " + to_hex(checksum)+"\n");
		return -1;
	}

	if ( (buf[3]+4) != n ) {
		flush();
		pc.print_debug("WRONG RETURN LENGHT\n");
		packetPrint(n, buf);
		return -1;
	}

	if(buf[4]!=0 ){
		pc.print_debug("STATUS ERROR \n");
		// bit 0
		 if ( !(buf[4] & 0x01) ) pc.print_debug("VOLTAGE OUT OF RANGE\n");	
		// bit 1
		else if ( !(buf[4] & 0x02) ) pc.print_debug("REQUIRED POSITION OUT OF RANGE\n");
		// bit 2
		else if ( !(buf[4] & 0x04) ) pc.print_debug("TEMPERATURE OUT OF RANGE\n");
		// bit 3
		else if ( !(buf[4] & 0x08) ) pc.print_debug("COMMAND OUT OF RANGE\n");
		// bit 4
		else if ( !(buf[4] & 0x10) ) pc.print_debug("CORRUPTED PACKAGE SENT - CRC DOES NOT MATCH\n");
		// bit 5
		else if ( !(buf[4] & 0x20) ) pc.print_debug("LOAD OUT OF RANGE\n");
		// bit 6
		else if ( !(buf[4] & 0x40) ) pc.print_debug("UNDEFINED OR MISSING COMMAND\n");
		// bit 7
		else if ( !(buf[4] & 0x80) ) pc.print_debug("GLITCH\n");
		return -1;
	}

	return 0;
}


// Packs data and sends it to the servo
// Dynamixel Communication 1.0 Protocol: Header, ID, Packet Length, Instruction, Parameter, 16bit CRC
int AX12A::send(const int& ID, const int& packetLength, unsigned char* parameters, const unsigned char& ins) {
	
	unsigned char buf[packetLength+6]; // Packet

	// Header
	buf[0] = 0xFF;
	buf[1] = 0xFF;

	// ID
	buf[2] = ID;

	// Packet Length
	buf[3] = packetLength+2;

	// Instruction
	buf[4] = ins;

	// Parameter
	for (int i=0; i < packetLength; i++) {
		buf[5+i] = parameters[i];
	}

	// Checksum
	buf[packetLength+5] = update_crc(buf, packetLength+5);

	// Transmit
	write(buf, packetLength+6);
	
	// Broadcast and Reply Lvl less than 2 do not reply
	if (ID == ID_Broadcast || ReturnLvl==0 || (ReturnLvl==1 && ins!=AX_INS_Read)) {
		return 0;	
	}

	
	// Read reply
	pc.print_debug("Reading reply\n");
	
	int n = read(reply_buf);
	if (n == 0) {
		pc.print_debug("Could not read status packet\n");
		return 0;
	}

	pc.print_debug("- Read " + itos(n) + " bytes\n");

	return statusError(reply_buf, n); // Return Error code
}


// dataPack sets the parameters in char array and returns length.
int AX12A::dataPack(const unsigned char& ins, unsigned char ** parameters, const int& address, const int& value /*=0*/){

	unsigned char* data; 
	
	int adrl = AddressLenght(address);

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
int AX12A::dataPush(const int& ID, const int& address, const int& value){
	flush(); // Flush reply for safety															
	
	unsigned char* parameters;
    int packetLength = dataPack(AX_INS_Write, &parameters, address, value);

    int ec = send(ID, packetLength, parameters, AX_INS_Write);
   	
   	delete[] parameters;

   	return ec;
}


// dataPull is a generic wrapper for single value GET instructions for public methods
int AX12A::dataPull(const int& ID, const int& address){
	flush(); // Flush reply	for safety														
	
	unsigned char* parameters;
    int packetLength = dataPack(AX_INS_Read, &parameters, address);

    int size = parameters[1];
   	
   	int ec = send(ID, packetLength, parameters, AX_INS_Read);

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
   		pc.print_debug("WRONG ID REPLIED\n");
   		return -1;
   	}
}



const unsigned char AX12A::TWO_BYTE_ADDRESSES[11] = { 0, 6, 8, 14, 30, 32, 34, 36, 38, 40, 48 };


/* ******************************** PRRIVATE METHODS END ************************************** */


