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

#include <Arduino.h>

class DnxHAL{

public:

  // Define the type for the constructor argument
  typedef HardwareSerial Port_t;
  typedef HardwareSerial* PortPtr_t;

  DnxHAL(DnxHAL::Port_t& port_in, long int baud_in, int return_lvl_in =1);
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
  virtual int spinCCW(int ID, int torque=1023) =0;
  virtual int spinCW(int ID, int torque=2047) =0;
  virtual int stopSpinning(int ID) =0;

  virtual int setJointMode(int ID);
  virtual int setWheelMode(int ID);
  virtual int enable(int ID);
  virtual int disable(int ID);

protected:
  
  template<class Type>
  inline unsigned char lobyte(Type num);
  template<class Type>
  inline unsigned char hibyte(Type num);
  template<class T1, class T2>
  inline uint16_t makeword(T1 num1, T2 num2);

  void flush();
  void write(unsigned char* buf, int n);
  int read(unsigned char* buf, int nMax=255);

  int angleScale(double angle);
  int getAddressLen(int address, const unsigned char * TWO_BYTE_ADDRESSES);
  void packetPrint(int bytes, unsigned char* buf);

  virtual int statusError(unsigned char* buf, int n) =0;
  virtual int send(int ID, int bytes, unsigned char* parameters, unsigned char ins) =0;

  virtual int dataPack(unsigned char ins, unsigned char ** parameters, int address, int value =0) =0;
  virtual int dataPush(int ID, int address, int value) =0;
  virtual int dataPull(int ID, int address) =0;

    void serialBaud(long int baud);

  // REPLY BUFFER - SIZE 256 Overflow should never occur no matter the number of servos - you only communicate with one ID
  // and others don't respond. ID_Broadcast does not reply as well 
    unsigned char reply_buf[256];   

    PortPtr_t port_;
    long int baud_;
    double bit_period_;
    int return_lvl_ = 1;

    bool debug_ = false;
};

// Control table: Only matching addresses are included
#define DNXHAL_ID             3
#define DNXHAL_BAUD                     4
#define DNXHAL_CW_LIMIT                 6
#define DNXHAL_CCW_LIMIT                8
#define DNXHAL_TORQUE_ENABLE        24

const unsigned char ID_Broadcast = 0xFE; // 254(0xFE) ID writes to all servos on the line

template<class Type>
inline unsigned char DnxHAL::lobyte(Type num){
  return (unsigned char)num;
}

template<class Type>
inline unsigned char DnxHAL::hibyte(Type num){
  return (unsigned char) (((uint16_t)num)>>8);
}

template<class T1, class T2>
inline uint16_t DnxHAL::makeword(T1 num1, T2 num2){
  return ( ((uint16_t)num1 & 0x00ff) | ( ((uint16_t)(num2) & 0x00ff) << 8 ) );
}



#endif  
