#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

int SwitchPin = 7;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(SwitchPin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.update();
  char val = Serial.read();

  if( val == '1'){
   Serial.println("Value : 1");
   digitalWrite(SwitchPin,HIGH);
   MPU_Setup();
   mpu.update(); 
  }
  else if (val == '0'){
    Serial.println("Value : 0");
    digitalWrite(SwitchPin,LOW);
  }

  Serial.print("X : ");
  Serial.print(mpu.getAngleX());
  Serial.print("\tY : ");
  Serial.print(mpu.getAngleY());
  Serial.print("\tZ : ");
  Serial.println(mpu.getAngleZ());
}

void MPU_Setup()
{
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    mpu.calcOffsets(true,true); // gyro and accelero
    Serial.println("Done!\n");
}
