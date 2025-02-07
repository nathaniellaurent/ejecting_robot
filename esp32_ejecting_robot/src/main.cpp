/***************************************************************************
 * Example sketch for the MPU6500_WE library
 *
 * This sketch shows how to get acceleration, gyroscocope and temperature
 * data from the MPU6500. In essence, the difference to the MPU9250 is the
 * missing magnetometer. The shall only show how to "translate" all other
 * MPU9250 example sketches for use of the MPU6500
 *
 * For further information visit my blog:
 *
 * https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
 * https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
 *
 ***************************************************************************/

#include <Wire.h>
#include <Deneyap_Servo.h>
#include <Arduino.h>

#include "Demo.h"
#include "Microros.h"
#include "Motor.h"

#define MPU6500_ADDR 0x68

#define USE_WIFI 1

#define AIN1 12
#define AIN2 13
#define BIN1 16
#define BIN2 17

unsigned long lastPing = millis();
unsigned long lastSuccess = millis();
bool currentStatus = true;

std::shared_ptr<MPU6500_WE> myMPU6500;
std::shared_ptr<Motor> motor1;
std::shared_ptr<Motor> motor2;

void setup()
{
    //   Serial.begin(115200);
    motor1 = std::make_shared<Motor>(Motor(AIN1, AIN2));
    motor2 = std::make_shared<Motor>(Motor(BIN1, BIN2));

    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);

    Wire.begin(23, 22);
    // Configure serial transport
    Serial.begin(115200);

    Error::setup();
    Error::display_error(1);
    myMPU6500 = std::make_shared<MPU6500_WE>(MPU6500_WE(MPU6500_ADDR));

    if (USE_WIFI)
    {

        Serial.println("Starting micro_ros...");

        Microros::setup(myMPU6500, motor1, motor2);
    }
    else
    {

        Demo::setup(motor1, motor2);

        if (!myMPU6500->init())
        {
            Serial.println("MPU6500 does not respond");
            Error::display_error(10);
        }
        else
        {
            Serial.println("MPU6500 is connected");
        }

        /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical
         * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the
         * MPU6500 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset
         * values. It assumes your MPU6500 is positioned flat with its x,y-plane. The more you deviate
         * from this, the less accurate will be your results.
         * The function also measures the offset of the gyroscope data. The gyroscope offset does not
         * depend on the positioning.
         * This function needs to be called at the beginning since it can overwrite your settings!
         */
        //   Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
        delay(10);
        myMPU6500->autoOffsets();
        myMPU6500->enableGyrDLPF();
        myMPU6500->setGyrDLPF(MPU6500_DLPF_6);
        myMPU6500->setSampleRateDivider(5);
        myMPU6500->setGyrRange(MPU6500_GYRO_RANGE_250);
        myMPU6500->setAccRange(MPU6500_ACC_RANGE_2G);
        myMPU6500->enableAccDLPF(true);
        myMPU6500->setAccDLPF(MPU6500_DLPF_6);
    }
}

void loop()
{

    delay(1);



    if (USE_WIFI)
    {

        if (millis() - lastPing > 3000)
        {
            lastPing = millis();
            Serial.println("Pinging agent...");
            if (Microros::ping())
            {
                currentStatus = true;
                lastSuccess = millis();

                Microros::spin_nodes();
            }
            else
            {
                currentStatus = false;
            }
        }
        else if (currentStatus == true)
        {
            Microros::spin_nodes();
        }

        if (millis() - lastSuccess > 10000)
        {
            motor1->stop();
            motor2->stop();
            Serial.println("Stopping micro_ros...");
            Error::display_error(14);
            Microros::shutdown();
            delay(1000);
            Error::display_error(13);
            Serial.println("Starting micro_ros...");
            Microros::setup(myMPU6500, motor1, motor2);
            lastPing = millis();
            lastSuccess = millis();
        }
    }

    else{

        xyzFloat gValue = myMPU6500->getGValues();
        xyzFloat gyr = myMPU6500->getGyrValues();
        float temp = myMPU6500->getTemperature();
        float resultantG = myMPU6500->getResultantG(gValue);

        Serial.println(gValue.z);
        

        if(gValue.z < 0.2){
            if(!Demo::isDemoRunning()){
            Demo::startDemo();
            }
            
        }
        if(gyr.z > 200){
            Demo::stopDemo();
        }
        Demo::spin();
    

        
    }

    //   Serial.println("Acceleration in g (x,y,z):");
    //   Serial.print(gValue.x);
    //   Serial.print("   ");
    //   Serial.print(gValue.y);
    //   Serial.print("   ");
    //   Serial.println(gValue.z);
    //   Serial.print("Resultant g: ");
    //   Serial.println(resultantG);

    //   Serial.println("Gyroscope data in degrees/s: ");
    //   Serial.print(gyr.x);
    //   Serial.print("   ");
    //   Serial.print(gyr.y);
    //   Serial.print("   ");
    //   Serial.println(gyr.z);

    //   Serial.print("Temperature in °C: ");
    //   Serial.println(temp);

    //   Serial.println("********************************************");
}
