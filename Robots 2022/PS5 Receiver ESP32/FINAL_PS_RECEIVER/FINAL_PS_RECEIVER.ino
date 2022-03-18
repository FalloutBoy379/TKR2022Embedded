#include <PS5BT.h>
#include <SPI.h>
#include "Wire.h"
#include <MPU6050_light.h>

//#include "RoboClaw_AVR.h"
#include <RoboClaw.h>


MPU6050 mpu(Wire);
unsigned long timer = 0;
#define KP_Gyro 1000
#define KI_Gyro 20
#define KD_Gyro 150

//Error variables
int current_gyro_error = 0, prev_gyro_error = 0, p_gyro_error = 0, i_gyro_error = 0, d_gyro_error = 0, P_gyro = 0, I_gyro = 0, D_gyro = 0;
//Others
int final_w = 0, compensation = 0, count_moving = 0, count_movingFlag = 0, pos = 0;

void gyroSetup();
int getGyroData();
int gyro_pid(int target, int current);

RoboClaw roboclaw (&Serial3, 10000);
//
//#define TIMER_INTERRUPT_DEBUG         0
//#define _TIMERINTERRUPT_LOGLEVEL_     0
//#define USE_TIMER_3     true
//#include "TimerInterrupt.h"

float M1Speed, M2Speed, M3Speed;
#define address1 128
#define address2 129

#define ps PS5.getButtonPress(PS)
#define cross PS5.getButtonPress(CROSS)
#define circle PS5.getButtonPress(CIRCLE)
#define square PS5.getButtonPress(SQUARE)
#define triangle PS5.getButtonPress(TRIANGLE)
#define up PS5.getButtonPress(UP)
#define down PS5.getButtonPress(DOWN)
#define left PS5.getButtonPress(LEFT)
#define right PS5.getButtonPress(RIGHT)
#define mic PS5.getButtonPress(MICROPHONE)
#define l1 PS5.getButtonPress(L1)
#define r1 PS5.getButtonPress(R1)
#define r3 PS5.getButtonPress(R3)
#define l3 PS5.getButtonPress(L3)
#define create PS5.getButtonPress(CREATE)
#define options PS5.getButtonPress(OPTIONS)
#define touchpad PS5.getButtonPress(TOUCHPAD)
#define disconn (r3 && l3) || ps

#define LX PS5.getAnalogHat(LeftHatX)
#define RX PS5.getAnalogHat(RightHatX)
#define LY PS5.getAnalogHat(LeftHatY)
#define RY PS5.getAnalogHat(RightHatY)

long int lastMillis;
USB Usb;

#define SPEED_LIMIT 30000

BTD Btd(&Usb);
PS5BT PS5(&Btd, 1);
uint8_t buttonData[3] = {0xFF, 0x0F, 0xF0};
int16_t JoystickData[4] = {0, 0, 0, 0};
int16_t lastJoystickData[4] = {0, 0, 0, 0};
long int lastTime = 0;

void setup() {
  //  ITimer3.init();
  //  ITimer3.attachInterruptInterval(100, TimerHandler3);
  Serial.begin(115200);
  roboclaw.begin(115200);
  //  rc_serialstart_3();
  if (Usb.Init() == -1) {
    Serial.print("Error in Controller interface!");
    while (1);
  }
  gyroSetup();
}

int comp;

void loop() {
  //  roboclaw.DutyM1(128, 20000);
  Usb.Task();
  getButtonValues();
  if(buttonData[0] == 32){
    comp++;
  }
  else if(buttonData[0] == 128){
    comp--;
  }
  Usb.Task();
  getJoystickValues();
  Usb.Task();
  mpu.update();
  int w = gyro_pid(0, mpu.getAngleZ());
  final_w = w;
  Serial.println(final_w);
  Usb.Task();
  if (millis() - lastTime > 100) {
    lastTime = millis();
    drive(JoystickData[0], JoystickData[1], final_w + (comp*100));
  }

  Usb.Task();
  //  sendData();
  resetButtonValues();
}

void resetButtonValues() {
  buttonData[0] = 0x00;
  buttonData[1] = 0x00;
  buttonData[2] = 0x00;
}

void getButtonValues() {
  buttonData[0] ^= (cross << 0 | circle << 1 | triangle << 2 | square << 3 | down << 4 | right << 5 | up << 6 | left << 7);
  buttonData[1] ^= (mic << 0 | l1 << 1 | options << 2 | touchpad << 3 | create << 4 | r3 << 5 | l3 << 6 | r1 << 7);
  buttonData[2] ^= (ps << 0 | disconn << 1 | 0 << 2 | 0 << 3 | 0 << 4 | 0 << 5 | 0 << 6 | 0 << 7);
  if (disconn == 1) {
    PS5.disconnect();
  }
}

void getJoystickValues() {
  JoystickData[0] = LX;
  JoystickData[1] = LY;
  //  JoystickData[2] = RX;
  //  JoystickData[3] = RY;
  JoystickData[0] = map(JoystickData[0], 0, 255, -32000, 32000);
  JoystickData[1] = map(JoystickData[1], 0, 255, -32000, 32000);
  //  JoystickData[2] = map(JoystickData[2], 0, 255, 0, 127);
  //  JoystickData[3] = map(JoystickData[3], 0, 255, 0, 127);
}

void sendData() {
  Serial.print(201);
  Serial.print(JoystickData[0]);
  Serial.print(202);
  Serial.print(JoystickData[1]);
  Serial.print(203);
  Serial.print(JoystickData[2]);
  Serial.print(204);
  Serial.print(JoystickData[3]);
  Serial.print(buttonData[0] + buttonData[1], BIN);
  Serial.print(buttonData[1], BIN);
  Serial.println(buttonData[2], BIN);
}

bool drive(int x, int y, int w) {
  M1Speed = 0.33 * w + 0.58 * y - 0.33 * x;
  M2Speed = 0.33 * w - 0.58 * y - 0.33 * x;
  M3Speed = -0.33 * w - 0.67 * x;
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  //  if (M1Speed > 32000) {
  //    M1Speed = 32000;
  //  }
  //  else if (M1Speed < -32000) {
  //    M1Speed = -32000;
  //  }
  //  if (M2Speed > 32000) {
  //    M2Speed = 32000;
  //  }
  //  else if (M2Speed < -32000) {
  //    M2Speed = -32000;
  //  }
  //  if (M3Speed > 32000) {
  //    M3Speed = 32000;
  //  }
  //  else if (M3Speed < -32000) {
  //    M3Speed = -32000;
  //  }
  Serial.print("\t");
  Serial.print(M1Speed);
  Serial.print("\t");
  Serial.print(-M2Speed);
  Serial.print("\t");
  Serial.println(M3Speed);
  roboclaw.DutyM1(address1, M1Speed);
  Usb.Task();
  roboclaw.DutyM2(address1, -M2Speed);
  Usb.Task();
  roboclaw.DutyM2(address2, M3Speed);
  Usb.Task();

  return true;
  //  driveM1(address1, M1Speed);
  //  driveM2(address1, -M2Speed);
  //  driveM2(address2, -M3Speed);
}


void gyroSetup() {
  Wire.begin();
  byte status = mpu.begin();
  while (status != 0) {
    Serial.println("FATALITY      -       Could not connect!!");
  }

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();
  Serial.println("Done!\n");
  delay(1000);
}


int gyro_pid(int target, int current) {
  current_gyro_error = current - target;

  p_gyro_error = current_gyro_error;
  i_gyro_error = prev_gyro_error + current_gyro_error;
  d_gyro_error = current_gyro_error - prev_gyro_error;

  P_gyro = KP_Gyro * p_gyro_error;
  I_gyro = KI_Gyro * i_gyro_error;
  D_gyro = KD_Gyro * d_gyro_error;

  prev_gyro_error = current_gyro_error;
  return -(P_gyro + I_gyro + D_gyro);
}
