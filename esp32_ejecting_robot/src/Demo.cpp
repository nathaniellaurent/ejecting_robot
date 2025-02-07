#include "Demo.h"

std::shared_ptr<Motor> Demo::motor1;
std::shared_ptr<Motor> Demo::motor2;

bool Demo::demoRunning = false;
unsigned long Demo::startTime = 0;

void Demo::setup(std::shared_ptr<Motor> passedMotor1, std::shared_ptr<Motor>  passedMotor2){
    motor1 = passedMotor1;
    motor2 = passedMotor2;
}

void Demo::startDemo(){
    demoRunning = true;
    startTime = millis();

}
void Demo::stopDemo(){
    demoRunning = false;

}
bool Demo::isDemoRunning(){
    return demoRunning;
}   

void Demo::spin(){
    if(demoRunning){
        if(millis() - startTime < 5000){
            motor1->setSpeed(1);
            motor2->setSpeed(1);
        }
        else if(millis() - startTime < 6000){
            motor1->setSpeed(-1);
            motor2->setSpeed(-1);
        }
        else if(millis() - startTime < 8000){
            motor1->setSpeed(-1);
            motor2->setSpeed(1);
        }
        else if(millis() - startTime < 13000){
            motor1->setSpeed(1);
            motor2->setSpeed(1);
        }
        else if(millis() - startTime < 17000){
            motor1->setSpeed(1);
            motor2->setSpeed(-1);
            
        }
        else if(millis() - startTime < 19000){
            stopDemo();
        }

    }
    else{
        motor1->setSpeed(0);
        motor2->setSpeed(0);
    }
}