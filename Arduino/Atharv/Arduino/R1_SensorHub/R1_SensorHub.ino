/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

int potPin = PB1;

MPU6050 mpu(Wire);
MPU6050 mpu2(Wire);
long timer = 0;

void setup() {
  pinMode(potPin,INPUT);
  
  Serial.begin(115200);
  Serial2.begin(115200);
  Wire.begin();
  mpu2.setAddress(0x69);
  byte status = mpu.begin();
  byte status2 = mpu2.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  while(status2!=0){};
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(true,true); // gyro and accelero
  mpu2.calcOffsets(true, true);
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();
  mpu2.update();
  int potValue = analogRead(potPin);
//    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
//    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
//    Serial.print("\tY: ");Serial.print(mpu.getAccY());
//    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
//  
//    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
//    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
//    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
//  
//    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
//    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    Serial2.write("*");
    delay(5);
    Serial2.write(getGyroAngle(0.5));
    
    Serial.print(mpu.getAngleZ());
    Serial.print("\t");
    Serial.println(mpu2.getAngleZ());
    Serial.print("\t");
    Serial.println(getGyroAngle(0.5));
    Serial.print("\n");
    Serial.println(potValue);
    
//    Serial.println(mpu.getAddress());
//    Serial.println(mpu2.getAddress());
//    Serial.println(F("=====================================================\n"));
  }

int getGyroAngle(float weight)
{
  int angle = (weight * mpu.getAngleZ()) + ((1-weight) * mpu2.getAngleZ());
  return angle;
}
