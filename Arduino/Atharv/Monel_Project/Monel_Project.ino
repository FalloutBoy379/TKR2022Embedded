#include <Servo.h>

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

//5v pin to vcc
//GND pin to GND
//SDA pin to A4
//SCL pin to A5

Servo myservo;
int val;

//GYRO Motor
int in1 = 2;
int in2 = 3;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  myservo.attach(2); // pin 2
 
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
 
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
 
  //GYRO Motor
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}


float filteredData = 0;

void loop() {
 
  mpu.update();
  float y = mpu.getAngleY();
//  filterGyroData(y,0.5);
  if((millis()-timer)>10){ // print data every 10ms
//Serial.print("X : ");
//Serial.print(mpu.getAngleX());
//Serial.print("\t Y : ");
//Serial.print(mpu.getAngleY());
//Serial.print("\t Z : ");
//Serial.println(mpu.getAngleZ());
timer = millis();    

  Serial.print("Y angle");
  Serial.println(y);
  y = map(y, 10, -10, 0, 60);     // scale it for use with the servo (value between 0 and 180)
//  Serial.println(y);
  myservo.write(y);                  // sets the servo position according to the scaled value
  gyromotor();
  }
}

void gyromotor(){
 
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //delay(1000);
}

//float filterGyroData(float Yangle ,float weight){
//  filteredData = ((1.0 - weight) * filteredData) + (weight * y);  
//}
