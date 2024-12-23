#include "Motor.h"


Motor::Motor(int pin1, int pin2) : pin1(pin1), pin2(pin2)
{

    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(sleepPin, OUTPUT);
    digitalWrite(sleepPin, HIGH);
}
void Motor::setSpeed(float speed)
{
    if (speed > 0)
    {
        analogWrite(pin1, speed * 255);
        analogWrite(pin2, 0);
        }
        else if (speed < 0)
        {
        analogWrite(pin1, 0);
        analogWrite(pin2, -speed * 255);
        }
        else
        {
        analogWrite(pin1, 0);
        analogWrite(pin2, 0);
    }
}
void Motor::stop()
{
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
}