#include "DnxHAL.h"

/* ============================================= MBED - PLATFORM SPECIFIC METHODS ============================================= */
#if DNX_PLATFORM_MBED

DnxHAL::DnxHAL(const DnxHAL::Port_t& port_in, int baud_in, int return_lvl_in /*=1*/) :
    port_(new mbed::Serial(port_in.tx, port_in.rx)), baud_(baud_in), bit_period_(1000000.0/baud_in), return_lvl_(return_lvl_in){

    // Set the baud rate of the port
    port_->baud(baud_);
}

DnxHAL::~DnxHAL(){
    delete port_;
}


// Clear input buffer
void DnxHAL::flush() {  
    while (port_->readable()) {
        port_->getc();
    }
}


// Write buffer to servo 
void DnxHAL::write(uint8_t* buf, int n) {
    for (int i=0; i < n; i++) {
        port_->putc(buf[i]);
    }

    if(debug_) fprintf(fp_debug_, "DnxHAL: about to clear buf\n\r");

    for(int i=0; i<n; ){
        if (port_->readable()){ 
            int inf = port_->getc();    //empty buffer because tx has written to rx (only in case of tx and rx connected)                                                               
            i++;                        //rate of the loop does not equal rate of communication
        }
    }

    if(debug_) fprintf(fp_debug_, "DnxHAL: buf cleared\n\r");
}


// Read reply returns payload length, 0 if error.
int DnxHAL::read(uint8_t* buf, int nMax /* =255 */) {           //check readBytesUntil()
    int n = 0;          // Bytes read
    int timeout = 0;    // Timeout

    while ((timeout < 100) && (n < nMax)) {
        if (port_->readable()) {
            buf[n] = port_->getc();
            n++;
            timeout = 0;
        }
        wait_us(bit_period_);                                                                   
        timeout++;      
    }

    return n;
}

/* ============================================= RASPBERRY PI - PLATFORM SPECIFIC METHODS ============================================= */
#elif DNX_PLATFORM_RPI

DnxHAL::DnxHAL(const DnxHAL::Port_t& port_in, int baud_in, int return_lvl_in /*=1*/) :
    port_(serialOpen(port_in.c_str(), baud_in)), baud_(baud_in), bit_period_(1000000.0/baud_in), return_lvl_(return_lvl_in) {

    if(baud_ > 230400){
        fprintf(fp_debug_, "Baud rate more than 230400 not supported\n\r");
        exit(EXIT_FAILURE);
    }

    if(port_ < 0){
        fprintf(fp_debug_, "Unable to open serial device %s: %s\n\r", port_in.c_str(), errno);
        exit(EXIT_FAILURE);
    }

    if (wiringPiSetup() == -1){
        fprintf(fp_debug_, "Unable to start wiringPi: %s\n\r", errno);
        exit(EXIT_FAILURE);
    }
}

DnxHAL::~DnxHAL(){
    serialClose(port_);
}


// Clear input buffer
void DnxHAL::flush() {  
    serialFlush(port_);
}


// Write buffer to servo 
void DnxHAL::write(uint8_t* buf, int n) {
       for (int i=0; i < n; i++) {
        serialPutchar(port_, buf[i]);
    }

    if(debug_) fprintf(fp_debug_, "DnxHAL: about to clear buf\n\r");

    for(int i=0; i<n; ){
        if ( serialDataAvail(port_) ){  
            int inf = serialGetchar(port_);      //empty buffer because tx has written to rx (only in case of tx and rx connected)                                                               
            i++;                                //rate of the loop does not equal rate of communication
        }
    }

    if(debug_) fprintf(fp_debug_, "DnxHAL: buf cleared\n\r");
}


// Read reply returns payload length, 0 if error.
int DnxHAL::read(uint8_t* buf, int nMax /* =255 */) {           
    int n = 0;          // Bytes read
    int timeout = 0;    // Timeout

    while ((timeout < 100) && (n < nMax)) {
        if ( serialDataAvail(port_)) {
            buf[n] = serialGetchar(port_);
            n++;
            timeout = 0;
        }
        else{
            delay(bit_period_);                                                                 
            timeout++;      
        }                                                           
    }

    return n;
}

#endif

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
int DnxHAL::readValue(int ID, int address){
    return dataPull(ID, address);
}


// CW - positive input, CCW - negative input
int DnxHAL::angleScale(double angle){
    
    // 2.61799387799 rad = 150 degrees
    int result = 512 - ((angle/2.61799387799)*512);
    // 0 is end CW and 1024 is end CCW

    if (result>1024){
        fprintf(fp_debug_, "DnxHAL: CCW out of range\n\r");
        return 1024;    
    } 

    else if (result<0){
        fprintf(fp_debug_, "DnxHAL: CW out of range\n\r");
        return 0;   
    }  
    
    else return result;
}


// Length of address
int DnxHAL::getAddressLen(int address, const uint8_t * TWO_BYTE_ADDRESSES) {
    for(int i=0; i<11; i++){
        if (address == TWO_BYTE_ADDRESSES[i]) return 2;
    }
    return 1;
}


// packetPrint
void DnxHAL::packetPrint(int bytes, uint8_t* buf) {
    if(!debug_) return;
    
    fprintf(fp_debug_, "DnxHAL PACKET: {");
    for (int i=0; i < bytes; i++) {
        fprintf(fp_debug_, "%x ", buf[i]);
    }
    fprintf(fp_debug_, " }\n\r");
}



