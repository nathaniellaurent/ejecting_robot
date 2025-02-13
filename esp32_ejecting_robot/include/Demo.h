#ifndef DEMO_H
#define DEMO_H

#include <Arduino.h>
#include <memory>
#include "Motor.h"

class Demo
{
    public:
        static void setup(std::shared_ptr<Motor> passedMotor1, std::shared_ptr<Motor>  passedMotor2);
        static void startDemo();
        static void stopDemo();
        static bool spin();
        static bool isDemoRunning();
        static void activate();
        static void trueActivate();

    private:
        static std::shared_ptr<Motor> motor1;
        static std::shared_ptr<Motor> motor2;
        static bool demoRunning;
        static unsigned long startTime;
};

#endif // DEMO_H
