#include "SerialXL320.h"


/* ******************************** PUBLIC METHODS ************************************** */

SerialXL320::SerialXL320(const DnxHAL::Port_t& port_in, long int baud_in, int return_level_in /*=1*/) :
    DnxHAL(port_in, baud_in, return_level_in){
    if(debug_) fprintf(fp_debug_, "XL320_SERIAL: Object attached to serial at baud rate %ld and bit period of %f us\n\r", baud_in, bit_period_);
}

SerialXL320::~SerialXL320(){}


// 0: 9600, 1:57600, 2:115200, 3:1Mbps
int SerialXL320::setBaud(int ID, int rate) {
	if ((rate > 3) || rate < 0) {
		if(debug_) fprintf(fp_debug_, "SerialXL320: Incorrect baud rate\n\r");
		return 1;
	}
	return dataPush(ID, XL_BAUD_RATE, rate);
}

// Set which commands return status; 0: None, 1: Read, 2: All.
int SerialXL320::setReturnLevel(int ID, int lvl) {
	return_lvl_=lvl;
	return dataPush(ID, XL_RETURN_LEVEL, return_lvl_);
}



// 1024 = -150 degrees CCW, 512 = 0 degrees (ORIGIN), 0 = +150 degrees CW
int SerialXL320::setGoalPosition(int ID, int angle){
	return dataPush(ID, XL_GOAL_POSITION_L, angle);
}

int SerialXL320::setGoalPosition(int ID, double angle){
	return dataPush(ID, XL_GOAL_POSITION_L, angleScale(angle));
}

int SerialXL320::setGoalVelocity(int ID, int velocity){
	return dataPush(ID, XL_GOAL_SPEED_L, velocity);
}

int SerialXL320::setGoalTorque(int ID, int torque){
	return dataPush(ID, XL_GOAL_TORQUE, torque);
}

int SerialXL320::setPunch(int ID, int punch){
	return dataPush(ID, XL_PUNCH, punch);
}

// Sets motor led colours. r = 1, g = 2, y = 3, b = 4, p = 5, c = 6, w = 7, o = 0
int SerialXL320::setLED(int ID, int colour){
    return dataPush(ID, XL_LED, colour);
}

int SerialXL320::setP(int ID, int value){
	return dataPush(ID, XL_P_GAIN, value);
}

int SerialXL320::setI(int ID, int value){
	return dataPush(ID, XL_I_GAIN, value);
}

int SerialXL320::setD(int ID, int value){
	return dataPush(ID, XL_D_GAIN, value);
}

int SerialXL320::spinCCW(int ID, int torque/*=1023*/){
    if(torque>1023) torque = 1023;
    if(torque<1) torque = 1;
    return dataPush(ID, XL_PRESENT_SPEED, torque);
}

int SerialXL320::spinCW(int ID, int torque/*=2047*/){
    if(torque>2047) torque = 2047;
    if(torque<1024) torque = 1024;
    return dataPush(ID, XL_PRESENT_SPEED, torque);
}

int SerialXL320::stopSpinning(int ID){
    return dataPush(ID, XL_PRESENT_SPEED, 0);
}


/* ******************************** PUBLIC METHODS END************************************** */


/* ******************************** PRIVATE METHODS ************************************** */


// Dynamixel Communication 2.0 Checksum
uint16_t SerialXL320::update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, const uint16_t& data_blk_size) {
    uint16_t i, j;
    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++) {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}


// Returns Length of Packet
int SerialXL320::PacketLength(uint8_t* buf) {
	// Header(3) + Reserved(1) + ID(1) +  Packet Length(?) + CRC(2)
	int Length = makeword(buf[5], buf[6]) + 7;
	return Length;
}


// Returns Length of Address
int SerialXL320::getAddressLen(int address) {
	return DnxHAL::getAddressLen(address, TWO_BYTE_ADDRESSES);
}


int SerialXL320::statusError(uint8_t* buf, int n) {

	// Minimum return length
	if (n < 11) {
		flush();
		if(debug_) fprintf(fp_debug_, "SerialXL320: READING CORRUPTION\n\r");
		return -1; 
	}

	if ((buf[0]!=0xFF)||(buf[1]!=0xFF)||(buf[2]!=0xFD)||(buf[3]!=0x00)) {
		flush();
		if(debug_) fprintf(fp_debug_, "SerialXL320: WRONG RETURN HEADER\n\r");
		packetPrint(n, buf);	
		return -1; 
	}

	int l = PacketLength(buf);
	if (l != n) {
		flush();
		if(debug_) fprintf(fp_debug_, "SerialXL320: WRONG RETURN LENGTH\n\r");
		packetPrint(n, buf);	
		return -1;
	}

	uint16_t CRC = update_crc(0, buf, n-2);
	uint16_t checksum = makeword(buf[n-2],buf[n-1]);
	if (CRC != checksum){ 
		flush();
		if(debug_) fprintf(fp_debug_, "SerialXL320: WRONG CHECKSUM\n\r");
		return -1;
	}


	if(buf[8]!=0 ){
		if(debug_) fprintf(fp_debug_, "SerialXL320: STATUS ERROR \n\r");
		if 		(buf[8] == 0x01) if(debug_) fprintf(fp_debug_, "SerialXL320: FAILED PROCESS OF INSTRUCTION\n\r");	
		else if (buf[8] == 0x02) if(debug_) fprintf(fp_debug_, "SerialXL320: UNDEFINED INSTRUCTION OR ACTION WITHOUT REG WRITE\n\r");
		else if (buf[8] == 0x03) if(debug_) fprintf(fp_debug_, "SerialXL320: CORRUPTED PACKAGE SENT - CRC DOES NOT MATCH\n\r");
		else if (buf[8] == 0x04) if(debug_) fprintf(fp_debug_, "SerialXL320: VALUE TO WRITE OUT OF RANGE\n\r");
		else if (buf[8] == 0x05) if(debug_) fprintf(fp_debug_, "SerialXL320: RECEIVED VALUE LENGTH SHORTER IN BYTES THAN REQUIRED FOR THIS ADDRESS\n\r");
		else if (buf[8] == 0x06) if(debug_) fprintf(fp_debug_, "SerialXL320: RECEIVED VALUE LENGTH LONGER IN BYTES THAN REQUIRED FOR THIS ADDRESS\n\r");
		else if (buf[8] == 0x07) if(debug_) fprintf(fp_debug_, "SerialXL320: READ_ONLY OR WRITE_ONLY ADDRESS\n\r");
		return -1;
	}

	return 0;
}


// Packs data and sends it to the servo
// Dynamixel Communication 2.0 Protocol: Header, Reserved, ID, Packet Length, Instruction, Parameter, 16bit CRC
int SerialXL320::send(int ID, int packetLenght, uint8_t* parameters, uint8_t ins) {
	uint8_t buf[255]; // Packet

	// Header
	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = 0xFD;

	// Reserved
	buf[3] = 0x00;

	// ID
	buf[4] = ID;

	// Packet Length
	buf[5] = lobyte(packetLenght+3);
	buf[6] = hibyte(packetLenght+3);

	// Instruction
	buf[7] = ins;

	// Parameter
	for (int i=0; i < packetLenght; i++) {
		buf[8+i] = parameters[i];
	}

	// Checksum
	uint16_t CRC = update_crc(0, buf, packetLenght+8);
	buf[packetLenght+8] = lobyte(CRC);
	buf[packetLenght+9] = hibyte(CRC);

	// Transmit
	write(buf, packetLenght+10);

	// Broadcast and Reply Lvl less than 2 do not reply
	if (ID == ID_Broadcast || return_lvl_==0 || (return_lvl_==1 && ins!=XL_INS_Read)) {
		return 0;	
	}

	// Read reply
	if(debug_) fprintf(fp_debug_, "SerialXL320: Reading reply\n\r");
	
	int n = read(reply_buf);
	if (n == 0) {
		if(debug_) fprintf(fp_debug_, "SerialXL320: Could not read status packet\n\r");
		return 0;
	}

	if(debug_) fprintf(fp_debug_, "SerialXL320: - Read %d bytes\n\r", n);

	return statusError(reply_buf, n); // Return Error code
}




// dataPack sets the parameters in char array and returns length.
int SerialXL320::dataPack(uint8_t ins, uint8_t ** parameters, int address, int value /*=0*/){

	uint8_t* data; 
	
	int adrl = getAddressLen(address);

	int size;
	if (ins == XL_INS_Write) size = adrl+2;
	else size = 4;

	data = new uint8_t[size];

	data[0] = lobyte(address);
	data[1] = hibyte(address);
	
	if (ins == XL_INS_Read){
		
		data[2] = lobyte(adrl);
		data[3] = hibyte(adrl);	
	}

	if (ins == XL_INS_Write){

		data[1+adrl] = hibyte(value);
		data[2] = lobyte(value);			// if adrl is 1, data[2] will be overwritten and again the correct packet will be sent
	}

	*parameters = data;

	return size;
}

// dataPush is a generic wrapper for single value SET instructions for public methods
int SerialXL320::dataPush(int ID, int address, int value){
	flush(); // Flush reply	for safety
	
	uint8_t* parameters;
    int packetLenght = dataPack(XL_INS_Write, &parameters, address, value);

    int ec = send(ID, packetLenght, parameters, XL_INS_Write);
   	
   	delete[] parameters;

   	return ec;
}


// dataPull is a generic wrapper for single value GET instructions for public methods
int SerialXL320::dataPull(int ID, int address){
	flush(); // Flush reply	for safety
	
	uint8_t* parameters;
    int packetLenght = dataPack(XL_INS_Read, &parameters, address);

    int size = parameters[2];
   // uint8_t buf[(11+size)] = {0};
   	
   	int ec = send(ID, packetLenght, parameters, XL_INS_Read);

   	delete[] parameters;
   	
   	if (ec != 0) {
   		return ec;
   	}

	//packetPrint(15, buf);
	if ( ((uint8_t)ID) == reply_buf[4] ){
   		if (size==2) return (unsigned int)makeword(reply_buf[9], reply_buf[10]);
   		else return (unsigned int)reply_buf[9];		
   	}

   	else{
   		if(debug_) fprintf(fp_debug_, "SerialXL320: WRONG ID REPLIED\n\r");
   		return -1;
   	}
}


const uint8_t SerialXL320::TWO_BYTE_ADDRESSES[11] = { 0, 6, 8, 15, 30, 32, 35, 37, 39, 41, 51 };


/* ******************************** PRIVATE METHODS END ************************************** */




