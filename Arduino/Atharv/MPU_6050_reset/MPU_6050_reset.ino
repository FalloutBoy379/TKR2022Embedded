/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

int SwitchPin = 7;

void setup() {
  pinMode(SwitchPin,OUTPUT);
  Serial.begin(115200);
  Wire.begin();
}

void loop() { 
  if (Serial.available())
  {
    char val = Serial.read();
    Serial.print("Val is: ");
    Serial.print("\t");
    Serial.println(val);
    mpu.update();
    
    Serial.print("X : ");
    Serial.print(mpu.getAngleX());
    Serial.print("\tY : ");
    Serial.print(mpu.getAngleY());
    Serial.print("\tZ : ");
    Serial.println(mpu.getAngleZ());
    timer = millis();  
  
    if (val == '1')
    {
      digitalWrite(SwitchPin,HIGH);
      mpu_reset();
      mpu.update();
    }
    else if (val == '0')
    {
      digitalWrite(SwitchPin,LOW);
      Serial.println("lmao");
    }
  }
}

void mpu_reset()
{
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n"); 
}
