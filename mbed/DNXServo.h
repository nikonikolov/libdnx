#ifndef DNXSERVO_H
#define DNXSERVO_H

#include "include.h"
#include <math.h>

/* Things to repair
	Reply buffer - size for 12 servos
	How to deal with setting the right return level in the DNX constructor
	Repair emptying the buffer when writing - read does not read single byte at a call
	Turn macros into functions	
*/


class DNXServo{

public:

	DNXServo(mbed::Serial* portIn, const int& baudIn, const int ReturnLvlIn = 1);

	virtual ~DNXServo();

    int SetID(const int& ID, const int& newID);
	int GetValue(const int& ID, const int& address);

    // virtual int Ping(int ID /*=1*/) =0;		// may be possible to put implementation here
	//int Ping(const int& ID);
    virtual int SetBaud	(const int& ID, const int& rate) =0;
    virtual int SetReturnLevel(const int& ID, const int& lvl) =0;
    virtual int SetLED(const int& ID, const int& colour) =0; 
	virtual int SetGoalPosition(const int& ID, const int& angle) =0;
	virtual int SetGoalPosition(const int& ID, const double& angle) =0;
	virtual int SetGoalVelocity(const int& ID, const int& velocity) =0;
	virtual int SetGoalTorque(const int& ID, const int& torque) =0;
	virtual int SetPunch(const int& ID, const int& punch) =0;

protected:
	
	void flush();
	void write(unsigned char* buf, const int& n);
	int read(unsigned char* buf, const int& nMax=255);

	int angleScale(const double& angle);
	int AddressLenght(const int& address, const unsigned char * TWO_BYTE_ADDRESSES);
	void packetPrint(const int& bytes, unsigned char* buf);
	
	virtual int statusError(unsigned char* buf, const int& n) =0;
	virtual int send(const int& ID, const int& bytes, unsigned char* parameters, const unsigned char& ins) =0;

	virtual int dataPack(const unsigned char& ins, unsigned char ** parameters, const int& address, const int& value =0) =0;
	virtual int dataPush(const int& ID, const int& address, const int& value) =0;
	virtual int dataPull(const int& ID, const int& address) =0;

	// REPLY BUFFER - SIZE 64 so that there is extra space available for extreme cases. NOTE that currently if 12 servos reply 
	//at once, buffer will overflow. That should not happen though, since read/write is done with one servo at a time
    unsigned char reply_buf[256];		

    mbed::Serial* port;
    int baud;
    double bitPeriod;
    int ReturnLvl = 2;

};

// Control table: Only matching addresses are included
#define DNXSERVO_ID 						3
#define DNXSERVO_BAUD 						4

// ID
const unsigned char ID_Broadcast = 0xFE; // 254(0xFE) ID writes to all servos on the line


// Util
//#define MAKEWORD(a, b) ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
//#define HIBYTE(w) ( (unsigned char) ( ((unsigned long)(w)) >> 8 ) )
#define MAKEWORD(a, b) ( ((unsigned short)(a) & 0x00ff) | ( ((unsigned short)(b) & 0x00ff) << 8 ) )
#define LOBYTE(w) ( (unsigned char)(w) )
#define HIBYTE(w) ( (unsigned char) ( ((unsigned short)(w)) >> 8 ) )


#endif //DNXSERVO_H