/* 

Dynamixel Communication Protocol 2.0 - implements communication for XL-320 servo motors
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


#ifndef SERIALXL320_H
#define SERIALXL320_H

#include "DnxHAL.h"

class SerialXL320 : public DnxHAL {
 
public:
 	
	SerialXL320(const DnxHAL::Port_t& port_in, int baud_in, int return_lvl_in=1);
    ~SerialXL320();
    
    int setBaud(int ID, int rate);
    int setReturnLevel(int ID, int lvl);

    //int setGoalPosition(int ID, double angle, bool cash=false); 
    //int setGoalPosition(int ID, int angle, bool cash=false);
	int setGoalPosition(int ID, double angle);
	int setGoalPosition(int ID, int angle);
	int setGoalVelocity(int ID, int velocity);
	int setGoalTorque(int ID, int torque);
	int setPunch(int ID, int punch);			// Sets the current to drive the motors
    int setLED(int ID, int colour);
    
    int setP(int ID, int value);
	int setI(int ID, int value);
	int setD(int ID, int value);

    int spinCCW(int ID, int torque=1023);
    int spinCW(int ID, int torque=2047);
    int stopSpinning(int ID);

private:
	
	uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, const uint16_t& data_blk_size);
	int PacketLength(uint8_t* buf);				// Returns length of packet

	int getAddressLen(int address);				// Returns length of an address in the Motor Control Table
	int statusError(uint8_t* buf, int n);
	int send(int ID, int bytes, uint8_t* parameters, uint8_t ins);

	int dataPack(uint8_t ins, uint8_t ** parameters, int address, int value =0);
	int dataPush(int ID, int address, int value);
	int dataPull(int ID, int address);
    
	static const uint8_t TWO_BYTE_ADDRESSES[11];

    static const uint8_t INS_REBOOT;
    static const uint8_t INS_STATUSRETURN;
    static const uint8_t INS_SYNCREAD;
    static const uint8_t INS_SYNCWRITE;
    static const uint8_t INS_BULKREAD;
    static const uint8_t INS_BULKWRITE;
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

#endif
