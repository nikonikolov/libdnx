#ifndef DNXSERVO_H
#define DNXSERVO_H

#include <Arduino.h>

class DNXServo{

public:

	DNXServo(HardwareSerial& port, long int baud);

	unsigned char getPort() const;

    int SetID(int ID, int newID);
	int GetValue(int ID, int address);

    // virtual int Ping(int ID /*=1*/) =0;		// may be possible to put implementation here
    virtual int SetBaud(int ID, int rate) =0;
    virtual int SetReturnLevel(int ID, int lvl) =0;
    virtual int SetLED(int ID, int colour) =0; 
	virtual int SetGoalPosition(int ID, int angle) =0;
	virtual int SetGoalVelocity(int ID, int velocity) =0;
	virtual int SetGoalTorque(int ID, int torque) =0;
	virtual int SetPunch(int ID, int punch) =0;

protected:
	

	int adr_length(int address, const unsigned char * two_byte);
	void packetPrint(int bytes, unsigned char* buf);
	
	void flush();
	void write(unsigned char* buf, int n);
	int read(int ID, unsigned char* buf, int nMax=255);

	virtual int statusError(unsigned char* buf, int n) =0;
	virtual int send(int ID, int bytes, unsigned char* parameters, unsigned char ins) =0;

	//int ping(int ID);
	virtual int dataPack(unsigned char ins, unsigned char ** parameters, int address, int value =0) =0;
	virtual int dataPush(int ID, int address, int value) =0;
	virtual int dataPull(int ID, int address) =0;

	// REPLY BUFFER
    unsigned char reply_buf[255];

    HardwareSerial* _port;
    unsigned char _port_num;
    long int _baud;
    double _bitPeriod;

};

// Control table: Only matching addresses are included
#define DNXServo_ID 						3

// ID
const unsigned char ID_Broadcast = 0xFE; // 254(0xFE) ID writes to all servos on the line


// Util
#define MAKEWORD(a, b) ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define LOBYTE(w) ( (unsigned char)(w) )
#define HIBYTE(w) ( (unsigned char) ( ((unsigned long)(w)) >> 8 ) )

#endif //DNXSERVO_H