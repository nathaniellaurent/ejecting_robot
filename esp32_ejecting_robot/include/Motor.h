#ifndef MOTOR_H 
#define MOTOR_H 

#include <Arduino.h>

#define sleepPin 33


class Motor{
    public:
        Motor(int pin1, int pin2);
        void setSpeed(float speed);
        void stop();

    private:
        int pin1;
        int pin2;


};

#endif // MOTOR_H
