#include "Error.h"


 void Error::setup()
{
    pinMode(errorPin1, OUTPUT);
    pinMode(errorPin2, OUTPUT);
    pinMode(errorPin3, OUTPUT);
    pinMode(errorPin4, OUTPUT);
    pinMode(ledPin, OUTPUT);
}

 void Error::display_error(uint16_t error_code)
{
   

    if(error_code % 2 == 1)
    {
        digitalWrite(errorPin1, HIGH);
    }
    else
    {
        digitalWrite(errorPin1, LOW);
    }
    if(error_code % 4 >= 2)
    {
        digitalWrite(errorPin2, HIGH);
    }
    else
    {
        digitalWrite(errorPin2, LOW);
    }
    if(error_code % 8 >= 4)
    {
        digitalWrite(errorPin3, HIGH);
    }
    else
    {
        digitalWrite(errorPin3, LOW);
    }
    if(error_code % 16 >= 8)
    {
        digitalWrite(errorPin4, HIGH);
    }
    else
    {
        digitalWrite(errorPin4, LOW);
    }
}

void Error::ledOn()
{
    digitalWrite(ledPin, HIGH);
}

void Error::ledOff()
{
    digitalWrite(ledPin, LOW);
}