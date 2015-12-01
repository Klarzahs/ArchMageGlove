// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-10-07 - initial release

/* ========================OUTPUT_TEAPOT====================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;

int offAX = 0;
int offAY = 0;
int offAZ = 0;
int offGX = 0;
int offGY = 0;
int offGZ = 0;
float angleAX = 0;
float angleAY = 0;
float angleAZ = 0;
float angleGX = 0;
float angleGY = 0;
float angleGZ = 0;
float angleFX = 0;
float angleFY = 0;
float angleFZ = 0;

long timestep;

void computeOffset(){
  for(int i = 0; i < 10; i++){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    offAX += ax;
    offAY += ay;
    offAZ += az;
    offGX += gx;
    offGY += gy;
    offGZ += gz;
  }
  offAX /= 10;
  offAY /= 10;
  offAZ /= 10;
  offGX /= 10;
  offGY /= 10;
  offGZ /= 10;
  Serial.print(offAX);
  Serial.print(offAY);
  Serial.print(offAZ);
  Serial.print(offGX);
  Serial.print(offGY);
  Serial.println(offGZ);
  delay(2000);
}

void initGyro(){
  // compute the acc angle and assign it to the gyro
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax -= offAX;
  ay -= offAY;
  az -= offAZ;
  angleFX = atan(ax / sqrt(ay*ay + az*az));
  angleFY = atan(ay / sqrt(ax*ax + az*az));
  angleFZ = atan(sqrt(ay*ay + ax*ax) / az);
  if(angleFX != angleFX) angleFX = 0;
  if(angleFY != angleFY) angleFY = 0;
  if(angleFZ != angleFZ) angleFZ = 0;
  Serial.print(angleFX);
  Serial.print(angleFY);
  Serial.println(angleFZ);
  delay(2000);
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    // compute the standard offset over 10 iterations
    computeOffset();
    initGyro();

    timestep = millis();
}

float SensitivityG = 131;
float alpha = 0.96f;

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    
    // substract the offset
    ax -= offAX;
    ay -= offAY;
    az -= offAZ;

    gx -= offGX;
    gy -= offGY;
    gz -= offGZ;

    //compute the acc angles
    angleAX = atan(ax / sqrt(ay*ay + az*az));
    angleAY = atan(ay / sqrt(ax*ax + az*az));
    angleAZ = atan(sqrt(ay*ay + ax*ax) / az);

    Serial.print("Angles Acc: ");
    Serial.print(angleAX);
    Serial.print(angleAY);
    Serial.println(angleAZ);

    //compute the gyro angles
    float dt = millis() - timestep;

    angleGX = angleFX + (gx / SensitivityG) * dt/1000;
    angleGY = angleFY + (gy / SensitivityG) * dt/1000;
    angleGZ = angleFZ + (gz / SensitivityG) * dt/1000;

    // complementary filter
    angleFX = alpha * angleGX + (1 - alpha) * angleAX;
    angleFY = alpha * angleGY + (1 - alpha) * angleAY;
    angleFZ = alpha * angleGZ + (1 - alpha) * angleAZ;

    timestep = millis();

    Serial.print(angleFX); Serial.print(", ");
    Serial.print(angleFY); Serial.print(", ");
    Serial.println(angleFZ);
}
