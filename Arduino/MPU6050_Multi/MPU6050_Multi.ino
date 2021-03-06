// Dr. Prashan Premaratne
//21/10/2014
//University of Wollongong
#include "Wire.h"
const int MPU=0x69;  // I2C address of the MPU-6050// This is valid only
//when AD0 is HIGH
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;


int Acc_ctrl_1 = 6;
int Acc_ctrl_2 = 7;

void setup() {                
      
  Wire.begin(); // wake up I2C bus
// set I/O pins to outputs
 
 
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  
    pinMode(Acc_ctrl_1, OUTPUT);  //S0
    pinMode(Acc_ctrl_2, OUTPUT);  //S1
    
    //S0=1, S1=2 and S2=4  so Y0= S0=0,S1=0,S2=0,  Y4=S0=0,S1=0,S2=1
}

void loop() {
    // control signal for First Accelerometer
    digitalWrite(Acc_ctrl_1, LOW);
    digitalWrite(Acc_ctrl_2, LOW);
    
    //Enable the MUX Chip 1 - Active Low
    readAccele();
    delay(5);
    
    // control signal for SECOND Accelerometer
    digitalWrite(Acc_ctrl_1, HIGH);
    digitalWrite(Acc_ctrl_2, LOW);
    
    readAccele();
    delay(5);
    
   // control signal for THIRD  Accelerometer
    digitalWrite(Acc_ctrl_1, LOW);
    digitalWrite(Acc_ctrl_2, HIGH);
    
    readAccele();
    delay(5);
    
    readAccele();
    delay(5);
    
}
void readAccele()
{
  Wire.beginTransmission(MPU);// I2C address code thanks to John Boxall
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(5);
}
