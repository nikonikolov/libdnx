# Dynamixel library

A cross-platform library for controlling Dynamixel Servo motors (http://support.robotis.com/beta/en/product/dxl_main.htm) 
from an embedded microcontroller.

## Supported platforms
Currently supported are 
- ARM mbed
- Raspberry Pi
- Arduino

## Supported servo models

The library implements both `Communication1.0` and `Communication2.0` Dynamixel protocols, so ideally (possibly with some extra tuning),
it should work for any Dynamixel servo which uses these protocols.

Servos which have been tested and confirmed to work:
- AX-12
- XL-320

## Platform selection

You should select the platform you are using when compiling the library with your code

### mbed
Define the macro `DNX_PLATFORM_MBED` at compile time

### Raspberry Pi
Define the macro `DNX_PLATFORM_RPI` at compile time
