/* Dynamixel Communication 1.0 */
/* Implements the dynamixel communication protocol for AX-12A */
#ifndef AX12A_H
#define AX12A_H

//#include <DNXServo.h>
#include "../DNXServo/DNXServo.h"

class AX12A : public DNXServo{
 
public:
 	
 	// Create Dynamixel Communication protocol 2.0
    AX12A(HardwareSerial& port, long int baud);

    //int Test(int ID);
    int SetBaud(int ID, int rate);
    int SetReturnLevel(int ID, int lvl);
    int SetLED(int ID, int colour); 
	int SetGoalPosition(int ID, int angle);
	int SetGoalVelocity(int ID, int velocity);
	int SetGoalTorque(int ID, int torque);
	int SetPunch(int ID, int punch);

private:

	unsigned char update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size);	

	int adr_length(int address);
	int statusError(unsigned char* buf, int n);
	int send(int ID, int bytes, unsigned char* parameters, unsigned char ins);

	int dataPack(unsigned char ins, unsigned char ** parameters, int address, int value =0);
	int dataPush(int ID, int address, int value);
	int dataPull(int ID, int address);
	
	static const unsigned char two_byte[11];

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


// INSTRUCTIONS
const unsigned char AX_INS_Ping = 0x01;         // Corresponding device ID command to check if packet reaches
const unsigned char AX_INS_Read = 0x02;         // Read command
const unsigned char AX_INS_Write = 0x03;        // Write command
const unsigned char AX_INS_RegWrite = 0x04;     // When receiving a write command packet data is not immediately written instead it goes into standby momentarily until action command arrives
const unsigned char AX_INS_Action = 0x05;       // Go command for Reg Write
const unsigned char AX_INS_Factory = 0x06;      // Reset All data to factory default settings
const unsigned char AX_INS_SyncWrite = 0x83;    // Write data from the same location and same size for multiple devices simultaneously

#endif  // AX12A_H