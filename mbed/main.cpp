#include "DNXServo.h"
#include "AX12A.h"
#include "XL320.h"
#include "include.h"

/* Global vars:
	PC pc - specifies connected pc for debugging purposes
	Serial deviceAX12A - connected to serial pins 9 and 10
	Serial deviceXL320 - connected to serial pins 13 and 14

*/
int main(){

	// Enable debugging
	pc.set_debug();
	
	int baud = 1000000;
	int ID = 1;

	/* 	Note that either of the following 2 statements automatically set the baud rate for the Serial port on the mbed
		However, the same baud rate is expected to be set in the physical servo (i.e. in the EEPROM table). If this is not
		the case, check int SetBaud(const int& ID, const int& rate); in the class header.
	*/
	XL320 SerialXL320(&deviceXL320, baud);		
	AX12A SerialAX12A(&deviceAX12A, baud);

	// Read the value for baud rate from the servo
	val = SerialAX12A.GetValue(ID, AX_BAUD_RATE);
	pc.print_debug("Baud rate read from id " + itos(ID) + " is " + itos(val) + "\n");

	// Set position for the servo
	SerialAX12A.SetGoalPosition(ID, 512);

	return 0;
}



