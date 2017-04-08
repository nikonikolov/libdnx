# Dynamixel library

A cross-platform library for controlling Dynamixel Servo motors (http://support.robotis.com/beta/en/product/dxl_main.htm) 
from an embedded microcontroller.

## Supported platforms
Currently supported are 
- ARM mbed
- Raspberry Pi
- Arduino

## Official Library
https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/c%2B%2B/src/dynamixel_sdk_linux/port_handler_linux.cpp


## Supported servo models

The library implements both `Communication1.0` and `Communication2.0` Dynamixel protocols, so ideally (possibly with some extra tuning),
it should work for any Dynamixel servo which uses these protocols.

Servos which have been tested and confirmed to work:
- AX-12
- XL-320

## Platform selection

The platform should be automatically selected by the compiler you are using - either `gcc` or `arm-none-eabi-gcc`

### Platform selection issues
In case you have any problems, make sure that you have the following macros defined by the Makefile or by the compiler, depending on the platform:

- mbed
You need the macro 'TARGET_LIKE_MBED'. It will usually be defined by the exported mbed makefile. Note that you must be compiling with `arm-none-eabi-gcc`.

- Raspberry Pi
You need the macro `__unix__`. It is a `gcc` macro and will be predefined by default if you are using `gcc` on a linux system.
