/* 

Dynamixel Communication Abstract class 
===========================================================================================

FUNCTIONALITY:
	1. Defines the basic functionality that a protocol implementation must support
	1. Connects to a Serial Port
	2. Provides Hardware Abstraction Layer - subclasses do not need to implement 
		hardware writing or reading associated with the serial port

-------------------------------------------------------------------------------------------

*/

#ifndef DNXHAL_H
#define DNXHAL_H

#include <cstdint>
#include <string>
using std::string;

// Include platform specific libraries
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstdlib>

class DnxHAL{

public:

	// Define the type for the constructor argument
    typedef string Port_t;

	DnxHAL(const DnxHAL::Port_t& port_in, int baud_in, int return_lvl_in =1);
	virtual ~DnxHAL();

    int setID(int ID, int newID);
	int getValue(int ID, int address);

    virtual int setBaud(int ID, int rate) =0;
    virtual int setReturnLevel(int ID, int lvl) =0;
    virtual int setLED(int ID, int colour) =0; 
	virtual int setGoalPosition(int ID, int angle) =0;
	virtual int setGoalPosition(int ID, double angle) =0;
	virtual int setGoalVelocity(int ID, int velocity) =0;
	virtual int setGoalTorque(int ID, int torque) =0;
	virtual int setPunch(int ID, int punch) =0;

protected:
	
	template<class Type>
	inline uint8_t lobyte(Type num);
	template<class Type>
	inline uint8_t hibyte(Type num);
	template<class T1, class T2>
	inline uint16_t makeword(T1 num1, T2 num2);

	void flush();
	void write(uint8_t* buf, int n);
	int read(uint8_t* buf, int nMax=255);

	int angleScale(double angle);
	int getAddressLen(int address, const uint8_t * TWO_BYTE_ADDRESSES);
	void packetPrint(int bytes, uint8_t* buf);

	virtual int statusError(uint8_t* buf, int n) =0;
	virtual int send(int ID, int bytes, uint8_t* parameters, uint8_t ins) =0;

	virtual int dataPack(uint8_t ins, uint8_t ** parameters, int address, int value =0) =0;
	virtual int dataPush(int ID, int address, int value) =0;
	virtual int dataPull(int ID, int address) =0;

	// REPLY BUFFER - SIZE 256 Overflow should never occur no matter the number of servos - you only communicate with one ID
	// and others don't respond. ID_Broadcast does not reply as well 
    uint8_t reply_buf[256];		

    int port_;
    int baud_;
    double bit_period_;
    int return_lvl_ = 1;

    bool debug_ = true;
    FILE* fp_debug_ = stdout;
};

// Control table: Only matching addresses are included
#define DNXHAL_ID 						3
#define DNXHAL_BAUD 					4

const uint8_t ID_Broadcast = 0xFE; // 254(0xFE) ID writes to all servos on the line

template<class Type>
inline uint8_t DnxHAL::lobyte(Type num){
	return (uint8_t)num;
}

template<class Type>
inline uint8_t DnxHAL::hibyte(Type num){
	return (uint8_t) (((uint16_t)num)>>8);
}

template<class T1, class T2>
inline uint16_t DnxHAL::makeword(T1 num1, T2 num2){
	return ( ((uint16_t)num1 & 0x00ff) | ( ((uint16_t)(num2) & 0x00ff) << 8 ) );
}



#endif	
