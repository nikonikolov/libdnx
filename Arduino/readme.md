Dynamixel library - Arduino
========

Installation
-------------------
Just copy the the folders in the `libraries` subdirectory of your Arduino
home folder. Check Arduino man page for details on installing custom libraries. 


Usage
-------------------

In your sketch you need to paster either `#include <XL320.h>` or `#include <AX12A.h>`,
or both, depending on what servos you want to communicate with.

Initializing a servo object:

`int baud_rate = 115200;
XL320 ServoXL320(Serial1, baud_rate); 	// Automatically starts Serial1 at the specified baud rate`

Note that this assumes your physical servo has the same baud rate already set.
If this is not the case, you can use the member method in the XL320 class 
`int SetBaud(const int& ID, const int& rate);` to change the baud rate.

Note as well that `ServoXL320` is actually bound to a serial port on the Arduino, not
to a particular servo. Therefore, you can hook up as many servos as you want to one port
of the Arduino.

Reading data from the RAM or the EEPROM tables of a servo:

`int address = XL_BAUD_RATE;
int ID = 1; 					// Use the ID of the servo you want to communicate to
int baud_rate_read = ServoXL320.GetValue(ID, address);`

Setting the position of the servo:
`ServoXL320.SetGoalPosition(ID, 512);`

`AX12A` has exactly the same usage. For more details, you can start from the header files.

