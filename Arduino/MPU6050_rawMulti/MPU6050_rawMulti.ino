#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"


MPU6050 accelgyro[3];

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup(){
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
   
   // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    for(int i = 0; i<3; i++){
      digitalWrite(6, i & 0x01);
      digitalWrite(7, (i>>1) & 0x01);
      delay(50);
  
      // initialize device
      accelgyro[i].initialize();
  
      // verify connection
      Serial.println("Testing device connections...");
      Serial.print(i);
      Serial.println(accelgyro[i].testConnection() ? "MPU6050 connection successful " : "MPU6050 connection failed ");
    }
   delay(1000);
}

void loop(){
  for(int i = 0; i < 3; i++){
   digitalWrite(6, i & 0x01);
   digitalWrite(7, (i>>1) & 0x01);
   delay(10);
   
   accelgyro[i].getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   delay(25);
   Serial.print(i);
   Serial.print(",");
   Serial.print(ax);
   Serial.print(",");
   Serial.print(ay);
   Serial.print(",");
   Serial.print(az);
   Serial.print(",");
   Serial.print(gx);
   Serial.print(",");
   Serial.print(gy);
   Serial.print(",");
   Serial.println(gz);
  }
}

