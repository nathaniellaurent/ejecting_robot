#include <Arduino.h>

const int errorPin1 = 21;
const int errorPin2 = 25;
const int errorPin3 = 26;
const int errorPin4 = 4;

class Error
{
public:
    static void setup();

    static void display_error(uint16_t error_code);
};