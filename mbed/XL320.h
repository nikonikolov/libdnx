/* Dynamixel Communication 2.0 */
/* Implements the dynamixel communication protocol for XL-320. */
#ifndef XL320_H
#define XL320_H

#include "DNXServo.h"


class XL320 : public DNXServo {
 
public:
 	
 	// Create Dynamixel Communication protocol 2.0
	XL320(const PinName tx, const PinName rx, const int& baudIn, const int ReturnLvlIn =1);

    ~XL320();
    
    int SetBaud(const int& ID, const int& rate);
    int SetReturnLevel(const int& ID, const int& lvl);

	int SetGoalPosition(const int& ID, const double& angle);
	int SetGoalPosition(const int& ID, const int& angle);
	int SetGoalVelocity(const int& ID, const int& velocity);
	int SetGoalTorque(const int& ID, const int& torque);
	int SetPunch(const int& ID, const int& punch);			// Sets the current to drive the motors
    
    int SetP(const int& ID, const int& value);
	int SetI(const int& ID, const int& value);
	int SetD(const int& ID, const int& value);

	//int Test(const int& ID);
    int Ping(const int& ID=1);
    int SetLED(const int& ID, const int& colour); 
    int Rainbow(const int& ID);

private:
	
	uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, const uint16_t& data_blk_size);
	int PacketLength(uint8_t* buf);				// Returns length of packet

	int AddressLength(const int& address);				// Returns length of an address in the Motor Control Table
	int statusError(uint8_t* buf, const int& n);
	int send(const int& ID, const int& bytes, uint8_t* parameters, const uint8_t& ins);

	int dataPack(const uint8_t& ins, uint8_t ** parameters, const int& address, const int& value =0);
	int dataPush(const int& ID, const int& address, const int& value);
	int dataPull(const int& ID, const int& address);
    
	static const uint8_t TWO_BYTE_ADDRESSES[11];

};

// EEPROM 
#define XL_MODEL_NUMBER_L 0
#define XL_MODEL_NUMBER_H 1
#define XL_VERSION 2
#define XL_ID 3
#define XL_BAUD_RATE 4
#define XL_RETURN_DELAY_TIME 5
#define XL_CW_ANGLE_LIMIT_L 6
#define XL_CW_ANGLE_LIMIT_H 7
#define XL_CCW_ANGLE_LIMIT_L 8
#define XL_CCW_ANGLE_LIMIT_H 9
#define XL_CONTROL_MODE 11
#define XL_LIMIT_TEMPERATURE 12
#define XL_DOWN_LIMIT_VOLTAGE 13
#define XL_UP_LIMIT_VOLTAGE 14
#define XL_MAX_TORQUE_L 15
#define XL_MAX_TORQUE_H 16
#define XL_RETURN_LEVEL 17
#define XL_ALARM_SHUTDOWN 18

// RAM 
#define XL_TORQUE_ENABLE 24
#define XL_LED 25
#define XL_D_GAIN 27
#define XL_I_GAIN 28
#define XL_P_GAIN 29
#define XL_GOAL_POSITION_L 30
#define XL_GOAL_SPEED_L 32
#define XL_GOAL_TORQUE 35
#define XL_PRESENT_POSITION 37
#define XL_PRESENT_SPEED 39
#define XL_PRESENT_LOAD 41
#define XL_PRESENT_VOLTAGE 45
#define XL_PRESENT_TEMPERATURE 46
#define XL_REGISTERED_INSTRUCTION 47
#define XL_MOVING 49
#define XL_HARDWARE_ERROR 50
#define XL_PUNCH 51

const uint8_t XL_ID_Broadcast = 0xFE; 	// 254(0xFE) ID writes to all servos on the line

// INSTRUCTIONS
const uint8_t XL_INS_Ping = 0x01;         // Corresponding device ID command to check if packet reaches
const uint8_t XL_INS_Read = 0x02;         // Read command
const uint8_t XL_INS_Write = 0x03;        // Write command
const uint8_t XL_INS_RegWrite = 0x04;     // When receiving a write command packet data is not immediately written instead it goes into standby momentarily until action command arrives
const uint8_t XL_INS_Action = 0x05;       // Go command for Reg Write
const uint8_t XL_INS_Factory = 0x06;      // Reset All data to factory default settings
const uint8_t XL_INS_Reboot = 0x08;       // Reboot device
const uint8_t XL_INS_StatusReturn = 0x55; // Instruction Packet response
const uint8_t XL_INS_SyncRead = 0x82;     // Read data from the same location and same size for multiple devices simultaneously
const uint8_t XL_INS_SyncWrite = 0x83;    // Write data from the same location and same size for multiple devices simultaneously
const uint8_t XL_INS_BulkRead = 0x92;     // Read data from the different locations and different sizes for multiple devices simultaneously
const uint8_t XL_INS_BulkWrite = 0x93;    // Write data from the different locations and different sizes for multiple devices simultaneously

#endif  // XL320_H