#ifndef DNXSERVO_H
#define DNXSERVO_H

#include "include.h"
#include <math.h>
#include <cstdint>


class DNXServo{

public:

	DNXServo(const PinName tx, const PinName rx, const int& baudIn, const int ReturnLvlIn =1);

	virtual ~DNXServo();

    int SetID(const int& ID, const int& newID);
	int GetValue(const int& ID, const int& address);

    virtual int SetBaud	(const int& ID, const int& rate) =0;
    virtual int SetReturnLevel(const int& ID, const int& lvl) =0;
    virtual int SetLED(const int& ID, const int& colour) =0; 
	virtual int SetGoalPosition(const int& ID, const int& angle) =0;
	virtual int SetGoalPosition(const int& ID, const double& angle) =0;
	virtual int SetGoalVelocity(const int& ID, const int& velocity) =0;
	virtual int SetGoalTorque(const int& ID, const int& torque) =0;
	virtual int SetPunch(const int& ID, const int& punch) =0;

protected:
	
	template<class Type>
	inline uint8_t lobyte(const Type& num);
	template<class Type>
	inline uint8_t hibyte(const Type& num);
	template<class T1, class T2>
	inline uint16_t makeword(const T1& num1, const T2& num2);

	void flush();
	void write(uint8_t* buf, const int& n);
	int read(uint8_t* buf, const int& nMax=255);

	int angleScale(const double& angle);
	int AddressLength(const int& address, const uint8_t * TWO_BYTE_ADDRESSES);
	void packetPrint(const int& bytes, uint8_t* buf);
	
	virtual int statusError(uint8_t* buf, const int& n) =0;
	virtual int send(const int& ID, const int& bytes, uint8_t* parameters, const uint8_t& ins) =0;

	virtual int dataPack(const uint8_t& ins, uint8_t ** parameters, const int& address, const int& value =0) =0;
	virtual int dataPush(const int& ID, const int& address, const int& value) =0;
	virtual int dataPull(const int& ID, const int& address) =0;

	// REPLY BUFFER - SIZE 256 Overflow should never occur no matter the number of servos - you only communicate with one ID
	// and others don't respond. ID_Broadcast does not reply as well 
    uint8_t reply_buf[256];		

    mbed::Serial* port;
    int baud;
    double bitPeriod;
    int ReturnLvl = 2;

};

// Control table: Only matching addresses are included
#define DNXSERVO_ID 						3
#define DNXSERVO_BAUD 						4

const uint8_t ID_Broadcast = 0xFE; // 254(0xFE) ID writes to all servos on the line

template<class Type>
inline uint8_t DNXServo::lobyte(const Type& num){
	return (uint8_t)num;
}

template<class Type>
inline uint8_t DNXServo::hibyte(const Type& num){
	return (uint8_t) (((uint16_t)num)>>8);
}

template<class T1, class T2>
inline uint16_t DNXServo::makeword(const T1& num1, const T2& num2){
	return ( ((uint16_t)num1 & 0x00ff) | ( ((uint16_t)(num2) & 0x00ff) << 8 ) );
}


#endif