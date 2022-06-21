/* 
 *  Author: Ansh Mehta
 *  Date: 05/06/2022
 *  Robot R2 Semi-Autonomous code for Online Game Rules  
 */

#include <RoboClaw.h>
#include <Wire.h>
#include <MPU6050_light.h>

//#########################################################################################################
//-------------------------------------------------- DECLARATIONS -----------------------------------------
//#########################################################################################################

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LIGHTS -------------------------------------------------
//---------------------------------------------------------------------------------------------------------

//Pin Definitions for LEDs
#define REDLED 4
#define GREENLED 8
#define BLUELED 7

void lights_setup();                              //Setup lights as output pins
void lights_write(int r, int g, int b);           //Write RGB values to LED                             

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- ULTRASOUND --------------------------------------------
//---------------------------------------------------------------------------------------------------------
int getPairDifference(int pairN);
int readUltrasound(int echoPin);

#include <NewPing.h>

#define SONAR_NUM 4      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(27, 26, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(29, 28, MAX_DISTANCE), 
  NewPing(31, 30, MAX_DISTANCE),
  NewPing(33, 32, MAX_DISTANCE)
};

#define KP_diffCorrection 4
#define KI_diffCorrection 0
#define KD_diffCorrection 2

int prev_error_diffCorrection;


#define KP_fenceFollowing 6
#define KI_fenceFollowing 0
#define KD_fenceFollowing 2.5
int prev_error_fenceFollowing;

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- MODE BUTTONS--------------------------------------------
//---------------------------------------------------------------------------------------------------------
#define BUTTON_ROUND1 A15
#define BUTTON_ROUND2 A13

#define ROUND1 0
#define ROUND2 1

int botMode = 0;
int currentPos = 1;

#define BUTTON_ROUND2_LED A14
#define BUTTON_ROUND1_LED A12

void button_setup();

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- GYRO --------------------------------------------------
//---------------------------------------------------------------------------------------------------------
MPU6050 mpu(Wire);
bool gyro_setup();
int16_t gyro_read();
int32_t gyro_pid(int16_t target, int16_t current);
int comp = 0;
#define GYROSPEED 50
int targetAngle = 0;
uint8_t KP_Gyro = 15;
uint8_t KI_Gyro = 0;
uint8_t KD_Gyro = 0;
//Error variables
long int current_gyro_error = 0, prev_gyro_error = 0, p_gyro_error = 0, i_gyro_error = 0, d_gyro_error = 0;
float P_gyro = 0, I_gyro = 0, D_gyro = 0;
//Others
long int final_w = 0;

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- BALL PICKING ------------------------------------------
//---------------------------------------------------------------------------------------------------------

//Define Pins for Ball Picking Motor and Encoder
#define DIR1 A6
#define DIR2 A7
#define PWM 11

#define EN1 2
#define EN2 3

//Define PID Constants
#define KP 5
#define KI 0.01
#define KD 0

//Define Pin for Solenoid for Piston actuation
#define PISTON A1

//PID to control height of Gripper
int prev_error = 0;
int PID(int currentValue, int targetValue);

volatile int count = 0;
int vel = 0;
int gripperState = 0;
//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- PS ---------------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool ps_read();
bool ps_setup();

#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))
#define pwm_range 115
#define PS_L1 0
#define PS_R1 1
#define PS_PS 5
#define PS_MIC 2
#define PS_L3 16
#define PS_R3 17
#define PS_TRIANGLE 6
#define PS_SQUARE 7
#define PS_CROSS 8
#define PS_CIRCLE 9
#define PS_UP 10
#define PS_LEFT 11
#define PS_DOWN 12
#define PS_RIGHT 13
#define PS_OPTIONS 14
#define PS_SHARE 15
#define PS_TOUCHPAD 18

int prevMillis = 0;
int xj1Offset = 0, yj1Offset = 0, xj2Offset = 0, yj2Offset = 0, calibrateFlag = 0, prevComp = 0;

int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0; //analog values(serially received from remote);
int butt[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //digital values(serially received from remote);

uint8_t RX[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0;
uint8_t TX[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
int flag[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;

int upflag = 0, downflag = 0;
//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- ROBOCLAW ---------------------------------------------
//---------------------------------------------------------------------------------------------------------
RoboClaw roboclaw(&Serial1, 10000);
bool roboclaw_setup();
#define address1 128
#define address2 129

//---------------------------------------------------------------------------------------------------------- -
//-------------------------------------------------- MOVEMENT ---------------------------------------------
//---------------------------------------------------------------------------------------------------------- -
bool drive(int x, int y, int w);
float M1Speed, M2Speed, M3Speed;
int Xflag = 0, loop3Counter = 0;
#define SAFESPEED 75
//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- OTHER ------------------------------------------------
//---------------------------------------------------------------------------------------------------------- -
float limitVar(float var, float target);
int loopVar = 0;

int R2BallPath = 0;
void interUART_setup();
void interUART_write(char data);
int xCounter=0;

//#########################################################################################################
//--------------------------------------------------  SETUP & LOOP  ---------------------------------------
//#########################################################################################################
void setup() {
  Serial.begin(115200);   // Intialize terminal serial port
  Serial.println("Starting...");
  lights_setup();
  button_setup();
  roboclaw_setup();
//  gyro_setup();
  ps_setup();
  interUART_setup();
  delay(1000);

  pinMode(EN1, INPUT_PULLUP);
  pinMode(EN2, INPUT_PULLUP);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(PISTON, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(EN1), encoder, RISING);
  interUART_write('7'); //Close Gripper
  interUART_write('8'); //Deploy Stacking
  interUART_write('0'); //Home Stacking
  interUART_write('1');
  interUART_write('2');
  interUART_write('3');
  interUART_write('4'); //Go to Lagori 4
  currentPos = 4;
//  setup_ultrasound(ECHO1);
//  setup_ultrasound(ECHO2);
//  setup_ultrasound(ECHO3);
//  setup_ultrasound(ECHO4);
}
void loop() {
  Serial.println(RX_count);
  if (RX_count == 1) {
    lights_write(0, 255, 0);
    ps_read();
    if (upflag == 1)
    {
      upflag = 1;
      downflag = 0;
      lights_write(0, 0, 255);
      vel = PID(count, 1650);
    }
    if (downflag == 1)
    {
      downflag = 1;
      upflag = 0;
      lights_write(255, 105, 180);
      vel = PID(count, 0);
    }
    targetAngle = targetAngle + comp * 10;
    comp = 0;
    int w = -gyro_pid(targetAngle, gyro_read());
    w = w;
    if (w > 127) {
      w = 127;
    }
    else if (w < -127) {
      w = -127;
    }
    if (botMode == ROUND2) {
      xj1 = -xj1;
      yj1 = -yj1;
      Serial.print("Current Position is: ");
      Serial.println(currentPos);
      lights_write(0, 0, 255);
      if (yj2 > 40) {
        interUART_write('a');
      }
      else if (yj2 < -40) {
        interUART_write('b');
      }
      else {
        interUART_write('c');
      }
    }
    Serial.print("encoder count:");
    Serial.print("\t");
    Serial.println(count);
    /* Serial.print(comp);
      Serial.print("\t");
      Serial.print(targetAngle);
      Serial.print("\t");
      Serial.print(gyro_read());
      Serial.print("\t");
      Serial.println(w);*/
//    if(R2BallPath == 0){
//      lights_write(255, 255, 0);
//    w = PID_diffCorrection(getPairDifference(2), 0);
//    xj1 = xj1 + PID_fenceFollowing((readUltrasound(2)+readUltrasound(3))/2, 15);
//    }
//    else if(R2BallPath == 1){
//      lights_write(255, 0, 255);
//      w = PID_diffCorrection(getPairDifference(1), 0);
//    yj1 = yj1 + PID_fenceFollowing((readUltrasound(0)+readUltrasound(1))/2, 30);
//    }
    xj1 = (cos(gyro_read())*xj1) - (sin(gyro_read()) * yj1);
    yj1 = (cos(gyro_read())*xj1) + (sin(gyro_read()) * yj1);
    drive(-yj1, xj1, -w);
  }
}

//#########################################################################################################
//-------------------------------------------------- DEFINITIONS ------------------------------------------
//#########################################################################################################


//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LIGHTS -------------------------------------------------
//---------------------------------------------------------------------------------------------------------
void lights_setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  digitalWrite(REDLED, HIGH);
  digitalWrite(GREENLED, LOW);
  digitalWrite(BLUELED, LOW);
}

void lights_write(int r, int g, int b) {
  analogWrite(REDLED, r);
  analogWrite(GREENLED, g);
  analogWrite(BLUELED, b);
}

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- GYRO --------------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool gyro_setup() {
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
  return true;
}

int16_t gyro_read() {
  mpu.update();
  return mpu.getAngleZ();
}

int32_t gyro_pid(int target, int current) {
  current_gyro_error = target - current;
  if (current_gyro_error > 20) {
    KP_Gyro = 6;
    KI_Gyro = 1;
    KD_Gyro = 8;
  }
  else {
    KP_Gyro = 7;
    KI_Gyro = 0;
    KD_Gyro = 1;
  }
  p_gyro_error = current_gyro_error;
  i_gyro_error = prev_gyro_error + current_gyro_error;
  d_gyro_error = current_gyro_error - prev_gyro_error;

  P_gyro = KP_Gyro * p_gyro_error;
  I_gyro = KI_Gyro * i_gyro_error;
  D_gyro = KD_Gyro * d_gyro_error;
  int corr = -(P_gyro + I_gyro + D_gyro);
  prev_gyro_error = current_gyro_error;
  if (current_gyro_error > 20) {
    if (corr > GYROSPEED) {
      corr = GYROSPEED;
    }
    else if (corr < -GYROSPEED) {
      corr = -GYROSPEED;
    }
  }
  return corr;
}

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- BALL_PICKING-------------------------------------------
//---------------------------------------------------------------------------------------------------------
void drive1(int vel) {
  if (vel > 0) {
    Serial.println(count);
    moveUpRack(vel);
  }
  else if (vel < 0) {
    vel = -vel;
    moveDownRack(vel);
    Serial.println(count);
  }
  else {
    moveUpRack(0);
  }
}

void moveUpRack(int vel) {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM, vel);
}

void moveDownRack(int vel) {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM, vel);
}
int PID(int currentValue, int targetValue) {
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error;
  int diff_error = error - prev_error;

  prev_error = error;
  return (KP * prop_error) + (KI * integral_error) + (KD * diff_error);
}

void encoder() {
  if (digitalRead(EN2)) {
    count++;
  }
  else {
    count--;
  }
}

//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- PS ---------------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool ps_setup() {
  UBRR3H = BAUDRATE >> 8;
  UBRR3L = BAUDRATE;
  UCSR3B = 0b10011000; //enable RXEN TXEN
  UCSR3C = 0b00000110;
}

ISR(USART3_RX_vect)
{
  RX_count = 1;
  RX_raw = UDR3;
  if ((RX_raw > 200) && (RX_raw < 255))
  {
    RX_ad = RX_raw;
    if ((RX_raw > 230) && (RX_raw < 247))
    {
      uint8_t r_temp0 = (RX_raw - 231);
      butt[r_temp0] = 1;
    }
  }
  else if ((RX_raw >= 0) && (RX_raw < 201))
  {
    uint8_t r_temp1 = (RX_ad - 201);
    if (r_temp1 < 16)
    {
      RX[r_temp1] = RX_raw;
    }
  }
}

bool ps_read() {
  RX_count = 0;
  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, pwm_range, (-pwm_range));
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));

  if (butt[PS_OPTIONS] == 1)
  {
    Serial.println("Options");
    butt[PS_OPTIONS] = 0;
  }
  if (butt[PS_SHARE] == 1)
  {
    Serial.println("Share");
    butt[PS_SHARE] = 0;
  }
  if (butt[PS_UP] == 1)
  {
    Serial.println("Up");
    if (botMode == ROUND2) {

      if (currentPos == 4) {
        interUART_write('5');
      }
      else if (currentPos == 3) {
        interUART_write('4');
        interUART_write('5');
      }
      else if (currentPos == 2) {
        interUART_write('3');
        interUART_write('4');
        interUART_write('5');
      }
      else if (currentPos == 1) {
        interUART_write('2');
        interUART_write('3');
        interUART_write('4');
        interUART_write('5');
      }
      currentPos = 5;

    }
    else {
      upflag = 1;
      downflag = 0;
    }
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    butt[PS_DOWN] = 0;
    if (botMode == ROUND2) {
      if (currentPos == 5) {
        interUART_write('4');
        interUART_write('3');
      }
      else if (currentPos == 4 || currentPos == 2) {
        Serial.println("noifboewibgrioengbiepbgviupergebiprugbb");
        interUART_write('3');
      }
      else if (currentPos == 1) {
        interUART_write('2');
        interUART_write('3');
      }
      currentPos = 3;
    }
    else {
      downflag = 1;
      upflag = 0;
    }
  }

  if (butt[PS_LEFT] == 1)
  {
    if (botMode == ROUND2) {

      if (currentPos == 5 || currentPos == 3) {
        interUART_write('4');
      }
      else if (currentPos == 2) {
        interUART_write('3');
        interUART_write('4');
      }
      else if (currentPos == 1) {
        interUART_write('2');
        interUART_write('3');
        interUART_write('4');
      }
      currentPos = 4;
    }
    else {
      digitalWrite(PISTON, HIGH);
    }
    Serial.println("left");

    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    if (botMode == ROUND2) {

      if (currentPos == 3 || currentPos == 1) {
        interUART_write('2');
      }
      else if (currentPos == 4) {
        interUART_write('3');
        interUART_write('2');
      }
      else if (currentPos == 5) {
        interUART_write('4');
        interUART_write('3');
        interUART_write('2');
      }
      currentPos = 2;
    }
    else {
      digitalWrite(PISTON, LOW);
    }
    Serial.println("right");

    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1)
  {
    comp--;
    Serial.println("square");
    butt[PS_SQUARE] = 0;
  }
  if (butt[PS_CIRCLE] == 1)
  {
    comp++;
    Serial.println("circle");
    butt[PS_CIRCLE] = 0;
  }
  if (butt[PS_TRIANGLE] == 1)
  {
    targetAngle = targetAngle - 90;
    butt[PS_TRIANGLE] = 0;
  }
  if (butt[PS_CROSS] == 1)
  {
    targetAngle = targetAngle + 90;
    butt[PS_CROSS] = 0;
  }
  if (butt[PS_L1] == 1)
  {
    if (botMode == ROUND1) {
      interUART_write('8'); //Deploy Stacking
      botMode = ROUND2;
      digitalWrite(BUTTON_ROUND2_LED, LOW);
      digitalWrite(BUTTON_ROUND1_LED, HIGH);
    }
    else {
      interUART_write('9'); //Deploy Stacking
      botMode = ROUND1;
      digitalWrite(BUTTON_ROUND1_LED, LOW);
      digitalWrite(BUTTON_ROUND2_LED, HIGH);
    }
    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {
    if (botMode == ROUND2) {
      interUART_write('0');
      currentPos = 1;
    }
    Serial.println("R1");
    butt[PS_R1] = 0;
  }
  if (butt[PS_MIC] == 1)
  {
    butt[PS_MIC] = 0;
  }
  if (butt[PS_PS] == 1)
  {
    butt[PS_PS] = 0;
  }
  if (butt[PS_L3] == 1)
  {
    if (botMode == ROUND2) {
      if (gripperState == 0) {
        gripperState = 1;
        interUART_write('6');
      }
      else if (gripperState == 1) {
        gripperState = 0;
        interUART_write('7');
      }
    }
    else {
      loopVar = 2;
    }
    Serial.println("L3");
    butt[PS_L3] = 0;
  }
  if (butt[PS_R3] == 1)
  {
    Serial.println("R3");
    butt[PS_R3] = 0;
  }

  if (vel > 255) {
    vel = 255;
  }
  else if (vel < -255) {
    vel = -255;
  }


  if (vel < 155 && vel > 0) {
    vel = 0;
  }
  else if (vel > -155 && vel < 0) {
    vel = 0;
  }
  drive1(vel);
}

//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- ROBOCLAW ---------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool roboclaw_setup() {
  roboclaw.begin(115200);
}
//---------------------------------------------------------------------------------------------------------- -
//-------------------------------------------------- MOVEMENT ---------------------------------------------
//---------------------------------------------------------------------------------------------------------- -
bool drive(int x, int y, int w) {
  M1Speed = 0.33 * w + 0.58 * y - 0.33 * x;
  M2Speed = 0.33 * w - 0.58 * y - 0.33 * x;
  M3Speed = -0.33 * w - 0.67 * x;
  M1Speed = limitVar(M1Speed, 30000);
  M2Speed = limitVar(M2Speed, 30000);
  M3Speed = limitVar(M3Speed, 30000);

  M1Speed = map(M1Speed, -127, 127, 0, 127);
  M2Speed = map(M2Speed, -127, 127, 0, 127);
  M3Speed = map(M3Speed, -127, 127, 0, 127);
  /*Serial.print("\t");
    Serial.print(M1Speed);
    Serial.print("\t");
    Serial.print(-M2Speed);
    Serial.print("\t");
    Serial.println(-M3Speed);*/
  roboclaw.ForwardBackwardM1(address1, -M1Speed);
  roboclaw.ForwardBackwardM2(address2, -M2Speed);
  roboclaw.ForwardBackwardM2(address1, -M3Speed);
  return true;
}


//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- OTHER ------------------------------------------------
//---------------------------------------------------------------------------------------------------------- -
void interUART_setup() {
  Serial2.begin(115200);
}

void interUART_write(char data) {
  Serial.println(data);
  Serial2.write(data);
}


float limitVar(float var, float target) {
  if (var >= target) {
    var = target;
  }
  else if (var <= -target) {
    var = -target;
  }
  return var;
}

void button_setup() {
  pinMode(BUTTON_ROUND1, INPUT);
  pinMode(BUTTON_ROUND2, INPUT);  
  pinMode(BUTTON_ROUND1_LED, OUTPUT);
  pinMode(BUTTON_ROUND2_LED, OUTPUT);

  digitalWrite(BUTTON_ROUND1_LED, LOW);
  digitalWrite(BUTTON_ROUND2_LED, LOW);
}

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- ULTRASOUND --------------------------------------------
//---------------------------------------------------------------------------------------------------------

int readUltrasound(int i){
  return sonar[i].ping_cm();
}

int PID_diffCorrection(int currentValue, int targetValue) {
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error;
  int diff_error = error - prev_error;

  prev_error_diffCorrection = error;
  return (KP_diffCorrection * prop_error) + (KI_diffCorrection * integral_error) + (KD_diffCorrection * diff_error);
}

int PID_fenceFollowing(int currentValue, int targetValue) {
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error;
  int diff_error = error - prev_error;

  prev_error_fenceFollowing = error;
  return (KP_fenceFollowing * prop_error) + (KI_fenceFollowing * integral_error) + (KD_fenceFollowing * diff_error);
}

int getPairDifference(int pairN){
  int r1 = 0;
  int r2 = 0;
  if(pairN == 1){
    r1 = readUltrasound(0);
    r2 = readUltrasound(1);
  }
  else if(pairN == 2){
    r1 = readUltrasound(2);
    r2 = readUltrasound(3);
  }

  return (r1-r2);
}
