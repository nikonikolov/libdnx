/* 

Dynamixel Communication Protocol 1.0 - implements communication for AX-12A servo motors
===========================================================================================

FUNCTIONALITY:
	1. Connects to a Serial Port
	2. Does not represent a single servo, rather represents a port and the associated protocol
	3. 

-------------------------------------------------------------------------------------------

PROTOCOL SUPPORT:
	1. Does not supoort all features of the protocol
	2. Only the basic features needed to control a single servo supported

-------------------------------------------------------------------------------------------

FRAMEWORK:
	1. Needs a Servo ID and corresponding value to be passed for the function being performed

-------------------------------------------------------------------------------------------

*/


#ifndef SERIALAX12_H
#define SERIALAX12_H

#include "DnxHAL.h"

class SerialAX12 : public DnxHAL{
 
public:
 	
	SerialAX12(const DnxHAL::Port_t& port_in, int baud_in, int return_level_in =1);
	~SerialAX12();

    int setBaud(int ID, int rate);
    int setReturnLevel(int ID, int lvl);

	int setGoalPosition(int ID, double angle); 
	int setGoalPosition(int ID, int angle);
	int setGoalVelocity(int ID, int velocity);
	int setGoalTorque(int ID, int torque);
	int setPunch(int ID, int punch);
    int setLED(int ID, int colour);

    int spinCCW(int ID, int torque=1023);
    int spinCW(int ID, int torque=2047);
    int stopSpinning(int ID);

private:

	uint8_t update_crc(uint8_t *data_blk_ptr, const uint16_t& data_blk_size);	

	int getAddressLen(int address);
	int statusError(uint8_t* buf, int n);
	int send(int ID, int packetLength, uint8_t* parameters, uint8_t ins);

	int dataPack(uint8_t ins, uint8_t** parameters, int address, int value =0);
	int dataPush(int ID, int address, int value);
	int dataPull(int ID, int address);
	
	static const uint8_t TWO_BYTE_ADDRESSES[11];

};

// EEPROM 
#define AX_MODEL_NUMBER 			0
#define AX_VERSION 					2
#define AX_ID 						3
#define AX_BAUD_RATE 				4
#define AX_RETURN_DELAY_TIME 		5
#define AX_CW_ANGLE_LIMIT 			6
#define AX_CCW_ANGLE_LIMIT	 		8
#define AX_LIMIT_TEMPERATURE 		11
#define AX_DOWN_LIMIT_VOLTAGE 		12
#define AX_UP_LIMIT_VOLTAGE 		13
#define AX_MAX_TORQUE	 			14
#define AX_RETURN_LEVEL 			16
#define AX_ALARM_LED 				17
#define AX_ALARM_SHUTDOWN 			18

// RAM 
#define AX_TORQUE_ENABLE 			24
#define AX_LED 						25
#define AX_CW_MARGIN 				26
#define AX_CCW_MARGIN 				27
#define AX_CW_SLOPE 				28
#define AX_CCW_SLOPE 				29
#define AX_GOAL_POSITION 			30
#define AX_GOAL_VELOCITY 			32
#define AX_LIMIT_TORQUE 			34
#define AX_PRESENT_POSITION			36
#define AX_PRESENT_SPEED 			38
#define AX_PRESENT_LOAD 			40
#define AX_PRESENT_VOLTAGE 			42
#define AX_PRESENT_TEMPERATURE 		43
#define AX_REGISTERED_INSTRUCTION 	44
#define AX_MOVING 					46
#define AX_EEPROM_LOCK				47
#define AX_PUNCH 					48

const uint8_t AX_ID_Broadcast = 0xFE; 	// 254(0xFE) ID writes to all servos on the line

// INSTRUCTIONS
const uint8_t AX_INS_Ping = 0x01;         // Corresponding device ID command to check if packet reaches
const uint8_t AX_INS_Read = 0x02;         // Read command
const uint8_t AX_INS_Write = 0x03;        // Write command
const uint8_t AX_INS_RegWrite = 0x04;     // When receiving a write command packet data is not immediately written instead it goes into standby momentarily until action command arrives
const uint8_t AX_INS_Action = 0x05;       // Go command for Reg Write
const uint8_t AX_INS_Factory = 0x06;      // Reset All data to factory default settings
const uint8_t AX_INS_SyncWrite = 0x83;    // Write data from the same location and same size for multiple devices simultaneously

#endif  // SERIALAX12_H
