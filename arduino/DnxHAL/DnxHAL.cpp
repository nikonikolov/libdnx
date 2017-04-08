#include "DnxHAL.h"

DnxHAL::DnxHAL(DnxHAL::Port_t& port_in, long int baud_in, int return_lvl_in /*=1*/) :
  port_(&port_in), baud_(baud_in), bit_period_(1000000.0/baud_in), return_lvl_(return_lvl_in) {

  Serial.println("Creating DnxHAL");
  port_->begin(baud_);
  Serial.println("Object created");
}

DnxHAL::~DnxHAL(){}


// Clear input buffer
void DnxHAL::flush() {  
  while (port_->available()) {
    port_->read();
  }
}


// Write buffer to servo 
void DnxHAL::write(unsigned char* buf, int n) {
  for (int i=0; i < n; i++) {
    port_->write(buf[i]);
  }

  Serial.println("DnxHAL: about to clear buf");

  for(int i=0; i<n; ){
    if (port_->available()){ 
      port_->read();          //empty buffer because tx has written to rx (only in case of tx and rx connected)                                                               
      i++;                    //rate of the loop does not equal rate of communication
    }
  }

  Serial.println("DnxHAL: buf cleared");
}


// Read reply returns payload length, 0 if error.
int DnxHAL::read(unsigned char* buf, int nMax /* =255 */) {           
  int n = 0;          // Bytes read
  int timeout = 0;    // Timeout

  while ((timeout < 100) && (n < nMax)) {
    if (port_->available()) {
      buf[n] = port_->read();
      n++;
      timeout = 0;
    }
    else{
      delayMicroseconds(bit_period_);                                                                 
      timeout++;      
    }                                                           
  }

  return n;
}

void DnxHAL::serialBaud(long int baud){
  baud_ = baud;
  bit_period_ = 1000000.0/baud_;
  port_->begin(baud_);
}


/* ============================================= PLATFORM INDEPENDENT METHODS ============================================= */

int DnxHAL::enable(int ID){
  return dataPush(ID, DNXHAL_TORQUE_ENABLE, 1);
};


int DnxHAL::disable(int ID){
  return dataPush(ID, DNXHAL_TORQUE_ENABLE, 0);
};


int DnxHAL::setWheelMode(int ID){
  dataPush(ID, DNXHAL_CW_LIMIT, 0);
  return dataPush(ID, DNXHAL_CCW_LIMIT, 0);
}

int DnxHAL::setJointMode(int ID){
  return dataPush(ID, DNXHAL_CCW_LIMIT, 1024);
}

// SetID
int DnxHAL::setID(int ID, int newID){
  return dataPush(ID, DNXHAL_ID, newID);
};


// Read Value from Control Table
int DnxHAL::getValue(int ID, int address){
  return dataPull(ID, address);
}


// CW - positive input, CCW - negative input
int DnxHAL::angleScale(double angle){
  
  // 2.61799387799 rad = 150 degrees
  int result = 512 - ((angle/2.61799387799)*512);
  // 0 is end CW and 1024 is end CCW

  if (result>1024){
    Serial.println("DnxHAL: CCW out of range");
    return 1024;    
  } 

  else if (result<0){
    Serial.println("DnxHAL: CW out of range");
    return 0;   
  }  
  
  else return result;
}


// Length of address
int DnxHAL::getAddressLen(int address, const unsigned char * TWO_BYTE_ADDRESSES) {
  for(int i=0; i<11; i++){
    if (address == TWO_BYTE_ADDRESSES[i]) return 2;
  }
  return 1;
}


// packetPrint
void DnxHAL::packetPrint(int bytes, unsigned char* buf) {
  if(!debug_) return;
  
  Serial.println("DnxHAL PACKET: {");
  for (int i=0; i < bytes; i++) {
    Serial.println(buf[i]);
  }
  Serial.println(" }");
}



