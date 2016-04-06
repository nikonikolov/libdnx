/* Dynamixel Communication 1.0 */
/* Implements the dynamixel communication protocol for AX-12A */
#ifndef AX12A_H
#define AX12A_H

#include "DNXServo.h"

class AX12A : public DNXServo{
 
public:
 	
 	// Create Dynamixel Communication protocol 2.0
	AX12A(const PinName tx, const PinName rx, const int& baudIn, const int ReturnLvlIn =1);

	~AX12A();

    int SetBaud(const int& ID, const int& rate);
    int SetReturnLevel(const int& ID, const int& lvl);

	int SetGoalPosition(const int& ID, const double& angle); 
	int SetGoalPosition(const int& ID, const int& angle);
	int SetGoalVelocity(const int& ID, const int& velocity);
	int SetGoalTorque(const int& ID, const int& torque);
	int SetPunch(const int& ID, const int& punch);

    int SetLED(const int& ID, const int& colour);

private:

	uint8_t update_crc(uint8_t *data_blk_ptr, const uint16_t& data_blk_size);	

	int AddressLength(const int& address);
	int statusError(uint8_t* buf, const int& n);
	int send(const int& ID, const int& packetLength, uint8_t* parameters, const uint8_t& ins);

	int dataPack(const uint8_t& ins, uint8_t** parameters, const int& address, const int& value =0);
	int dataPush(const int& ID, const int& address, const int& value);
	int dataPull(const int& ID, const int& address);
	
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

#endif  // AX12A_H