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

#include "Microros.h"

#define MPU6500_ADDR 0x68


unsigned long lastPing = millis();
unsigned long lastSuccess = millis();
bool currentStatus = true;

std::shared_ptr<MPU6500_WE> myMPU6500;

void setup()
{
    //   Serial.begin(115200);

    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);

    Wire.begin(23, 22);
    // Configure serial transport
    Serial.begin(115200);

    Error::setup();
    Error::display_error(1);

    Serial.println("Starting micro_ros...");

    myMPU6500 = std::make_shared<MPU6500_WE>(MPU6500_WE(MPU6500_ADDR));


    Microros::setup(myMPU6500);
}

void loop()
{
    delay(1);
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

    if (millis() - lastSuccess > 5000)
    {
        Serial.println("Stopping micro_ros...");
        Error::display_error(14);
        Microros::shutdown();
        delay(1000);
        Error::display_error(13);
        Serial.println("Starting micro_ros...");
        Microros::setup(myMPU6500);
        lastPing = millis();
        lastSuccess = millis();
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
