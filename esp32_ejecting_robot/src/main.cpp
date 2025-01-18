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

unsigned long lastPing = millis();
bool currentStatus = true;

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

    Microros::setup();

    

}

void loop()
{
    delay(1);
    if(millis() - lastPing > 1000)
    {
        lastPing = millis();
        if(Microros::ping())
        {
            Microros::spin_nodes();
        }
        else{
            currentStatus = true;
        }
    }
    else if(currentStatus = true){
        Microros::spin_nodes();
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
