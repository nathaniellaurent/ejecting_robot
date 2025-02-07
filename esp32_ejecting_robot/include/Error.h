#ifndef ERROR_H 
#define ERROR_H 

#include <Arduino.h>

const int errorPin1 = 21;
const int errorPin2 = 25;
const int errorPin3 = 26;
const int errorPin4 = 4;
const int ledPin = 27;


class Error
{
public:
    static void setup();

    static void display_error(uint16_t error_code);
    static void ledOn();
    static void ledOff();
};

#endif // ERROR_H