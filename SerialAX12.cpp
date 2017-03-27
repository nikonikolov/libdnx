#include "SerialAX12.h"


SerialAX12::SerialAX12(const DnxHAL::Port_t& port_in, int baud_in, int return_level_in /*=1*/) :
	DnxHAL(port_in, baud_in, return_level_in){
	PRINT_DEBUG("SerialAX12: Object attached to serial at baud rate %ld and bit period of %f us", baud_, bit_period_);
}

SerialAX12::~SerialAX12(){}



// 1: 1Mbps, 3: 500 000, 4: 400 000, 7: 250 000, 9: 200 000, 16: 115200, 34: 57600, 103: 19200, 207: 9600
int SerialAX12::setBaud(int ID, int rate) {
	if ( rate != 1 && rate != 3 && rate != 4 && rate != 7 && rate != 9 && rate != 16  && rate != 34 && rate != 103 && rate != 207 ) {
		PRINT_DEBUG("SerialAX12: Incorrect baud rate");
		return 1;
	}

	return dataPush(ID, AX_BAUD_RATE, rate);
}

// Set which commands return status; 0: None, 1: Read, 2: All.
int SerialAX12::setReturnLevel(int ID, int lvl) {
	return_lvl_ = lvl;
	return dataPush(ID, AX_RETURN_LEVEL, return_lvl_);
}



// 1024 = -150 degrees CCW, 512 = 0 degrees (ORIGIN), 0 = +150 degrees CW
int SerialAX12::setGoalPosition(int ID, int angle){
	return dataPush(ID, AX_GOAL_POSITION, angle);
}

int SerialAX12::setGoalPosition(int ID, double angle){
	return dataPush(ID, AX_GOAL_POSITION, angleScale(angle));
}

int SerialAX12::setGoalVelocity(int ID, int velocity){
	return dataPush(ID, AX_GOAL_VELOCITY, velocity);
}

int SerialAX12::setGoalTorque(int ID, int torque){
	return dataPush(ID, AX_MAX_TORQUE, torque);
}

int SerialAX12::setPunch(int ID, int punch){
	return dataPush(ID, AX_PUNCH, punch);
}

// Turn LED on (0x01) and off (0x00)
int SerialAX12::setLED(int ID, int value){
	return dataPush(ID, AX_LED, value);
}

int SerialAX12::spinCCW(int ID, int torque/*=1023*/){
  if(torque>1023) torque = 1023;
  if(torque<1) torque = 1;
  return dataPush(ID, AX_PRESENT_SPEED, torque);
}

int SerialAX12::spinCW(int ID, int torque/*=2047*/){
  if(torque>2047) torque = 2047;
  if(torque<1024) torque = 1024;
  return dataPush(ID, AX_PRESENT_SPEED, torque);
}

int SerialAX12::stopSpinning(int ID){
  return dataPush(ID, AX_PRESENT_SPEED, 0);
}


/* ******************************** PUBLIC METHODS END ************************************** */

/* ******************************** PRIVATE METHODS ************************************** */


// Dynamixel Communication 1.0 Checksum
uint8_t SerialAX12::update_crc(uint8_t *data_blk_ptr, const uint16_t& data_blk_size) {
  
  uint8_t crc_accum=0;
  
  // Header bytes (0xFF, 0xFF) do not get included in the checksum  
  for(uint8_t i = 2; i < data_blk_size; i++) {
      crc_accum += data_blk_ptr[i];
  }
  
  return ~(crc_accum);
}


// Return Length of Address
int SerialAX12::getAddressLen(int address) {
	return DnxHAL::getAddressLen(address, TWO_BYTE_ADDRESSES);
}


int SerialAX12::statusError(uint8_t* buf, int n) {
	
	// Minimum return length
	if (n < 6) {
		flush();
		PRINT_DEBUG("SerialAX12: READING CORRUPTION");
		return -1; 
	}

	if ((buf[0]!=0xFF)||(buf[1]!=0xFF)) {
		flush();
		PRINT_DEBUG("SerialAX12: WRONG RETURN HEADER");
		packetPrint(n, buf);
		return -1; 
	}

	uint8_t checksum=update_crc(buf, n-1);
	// The last byte does not get included in the checksum
	if(checksum != buf[n-1]){
		flush();
			PRINT_DEBUG("SerialAX12: WRONG RETURN CHECKSUM");
			packetPrint(n, buf);
			PRINT_DEBUG("SerialAX12: CHECKSUM READ IS %X", checksum);
		return -1;
	}

	if ( (buf[3]+4) != n ) {
		flush();
		PRINT_DEBUG("SerialAX12: WRONG RETURN LENGHT");
		packetPrint(n, buf);
		return -1;
	}

	if(buf[4]!=0 ){
		PRINT_DEBUG("SerialAX12: STATUS ERROR ");
		// bit 0
		 if ( !(buf[4] & 0x01) ) PRINT_DEBUG("SerialAX12: VOLTAGE OUT OF RANGE");	
		// bit 1
		else if ( !(buf[4] & 0x02) ) PRINT_DEBUG("SerialAX12: REQUIRED POSITION OUT OF RANGE");
		// bit 2
		else if ( !(buf[4] & 0x04) ) PRINT_DEBUG("SerialAX12: TEMPERATURE OUT OF RANGE");
		// bit 3
		else if ( !(buf[4] & 0x08) ) PRINT_DEBUG("SerialAX12: COMMAND OUT OF RANGE");
		// bit 4
		else if ( !(buf[4] & 0x10) ) PRINT_DEBUG("SerialAX12: CORRUPTED PACKAGE SENT - CRC DOES NOT MATCH");
		// bit 5
		else if ( !(buf[4] & 0x20) ) PRINT_DEBUG("SerialAX12: LOAD OUT OF RANGE");
		// bit 6
		else if ( !(buf[4] & 0x40) ) PRINT_DEBUG("SerialAX12: UNDEFINED OR MISSING COMMAND");
		// bit 7
		else if ( !(buf[4] & 0x80) ) PRINT_DEBUG("SerialAX12: GLITCH");
		return -1;
	}

	return 0;
}


// Packs data and sends it to the servo
// Dynamixel Communication 1.0 Protocol: Header, ID, Packet Length, Instruction, Parameter, 16bit CRC
int SerialAX12::send(int ID, int packetLength, uint8_t* parameters, uint8_t ins) {
	
	uint8_t buf[packetLength+6]; // Packet

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
	//PRINT_DEBUG("Packet written");
	
	// Broadcast and Reply Lvl less than 2 do not reply
	if (ID == DNX_ID_BROADCAST || return_lvl_==0 || (return_lvl_==1 && ins!=AX_INS_Read)) {
		return 0;	
	}

	
	// Read reply
	PRINT_DEBUG("SerialAX12: Reading reply");	

	int n = read(reply_buf);
	if (n == 0) {
		PRINT_DEBUG("SerialAX12: Could not process status packet - zero bytes read");
		//throw NoBytesRead();
		return 0;
	}

	PRINT_DEBUG("SerialAX12: - Read %d bytes", n);

	return statusError(reply_buf, n); // Return Error code
}


// dataPack sets the parameters in char array and returns length.
int SerialAX12::dataPack(uint8_t ins, uint8_t ** parameters, int address, int value /*=0*/){

	uint8_t* data; 
	
	int adrl = getAddressLen(address);

	int size;
	if (ins == AX_INS_Write) size = adrl+1;
	else size = 2;

	data = new uint8_t[size];
	data[0] = lobyte(address);
	
	if (ins == AX_INS_Read){
		
		data[1] = lobyte(adrl);	
	}

	if (ins == AX_INS_Write){

		data[adrl] = hibyte(value);
		data[1] = lobyte(value);			// if adrl is 1, data[2] will be overwritten and again the correct packet will be sent
	}

	*parameters = data;

	return size;
}


// dataPush is a generic wrapper for single value SET instructions for public methods
int SerialAX12::dataPush(int ID, int address, int value){
	flush(); // Flush reply for safety															
	
	uint8_t* parameters;
    int packetLength = dataPack(AX_INS_Write, &parameters, address, value);

    int ec = send(ID, packetLength, parameters, AX_INS_Write);
   	
   	delete[] parameters;

   	return ec;
}


// dataPull is a generic wrapper for single value GET instructions for public methods
int SerialAX12::dataPull(int ID, int address){
	flush(); // Flush reply	for safety														
	
	uint8_t* parameters;
    int packetLength = dataPack(AX_INS_Read, &parameters, address);

    int size = parameters[1];
   	/*
   	int ec;
   	try{
	   	ec = send(ID, packetLength, parameters, AX_INS_Read);
   	}
   	catch(NoBytesRead& e){
   		PRINT_DEBUG("SerialAX12: Exception %s", e.what());
	   	delete[] parameters;
   		return -1;
   	}
   	*/
   	int ec = send(ID, packetLength, parameters, AX_INS_Read);

   	delete[] parameters;
   	
   	if (ec != 0) {
   		return ec;
   	}

	//packetPrint(15, buf);
	if ( ((uint8_t)ID) == reply_buf[2] ){
   		if (size==2) return (unsigned int)makeword(reply_buf[5], reply_buf[6]);
   		else return (unsigned int)reply_buf[5];	
   	}

   	else{
   		PRINT_DEBUG("SerialAX12: WRONG ID %X REPLIED", reply_buf[2]);
   		return -1;
   	}
}



const uint8_t SerialAX12::TWO_BYTE_ADDRESSES[11] = { 0, 6, 8, 14, 30, 32, 34, 36, 38, 40, 48 };


/* ******************************** PRRIVATE METHODS END ************************************** */


