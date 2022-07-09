#include <RoboClaw.h>
#include <Wire.h>
#include <MPU6050_light.h>

//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- ROBOCLAW ---------------------------------------------
//---------------------------------------------------------------------------------------------------------
RoboClaw roboclaw(&Serial1, 10000);
bool roboclaw_setup();
#define address1 128
#define address2 129

bool drive(int x, int y, int w);
bool drive(int x, int y, int w, int theta);
const float DEG2RAD = PI / 180;
float M1Speed, M2Speed, M3Speed;
int Xflag = 0, loop3Counter = 0;
#define SAFESPEED 75

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
//-------------------------------------------------- PS ---------------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool ps_read();
bool ps_setup();

unsigned long disconnected_counter = 0;

#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))
#define pwm_range 127
#define PS_L1 0
#define PS_R1 1
#define PS_PS 5
//#define PS_MIC 2
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
#define PS_TOUCHPAD 2

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
//------------------------------------------------- LIGHTS -------------------------------------------------
//---------------------------------------------------------------------------------------------------------

//Pin Definitions for LEDs
#define REDLED 10
#define GREENLED 9
#define BLUELED 8

int rgb[3];

void lights_setup();                              //Setup lights as output pins
void lights_write(int r, int g, int b);           //Write RGB values to LED


//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- BALL PICKING ------------------------------------------
//---------------------------------------------------------------------------------------------------------

//Define Pins for Ball Picking Motor and Encoder
#define DIR1 6
#define DIR2 4
#define PWM 5

#define TOP 3
#define BOTTOM 2

//Define PID Constants
#define KP_rack 10
#define KI_rack 0.01
#define KD_rack 0

//Define Pin for Solenoid for Piston actuation
#define PISTON 22

//PID to control height of Gripper
int prev_error_rack = 0;
int PID_rack(int currentValue, int targetValue);

volatile int count = 0;
int vel = 0;
int gripperState = 0;
int RackFlag = 0;

//#define UPPER_LIMIT 1733
#define UPPER_LIMIT 1680
#define LOWER_LIMIT 0



//STACKING
void interUART_setup();
void interUART_write(char data);
#define ROUND1 0
#define ROUND2 1

int botMode = 0;
int currentPos = 1;





void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  lights_setup();
  lights_write(120, 255, 120);
  roboclaw_setup();
  gyro_setup();
  ps_setup();
  interUART_setup();
  pinMode(TOP, INPUT);
  pinMode(BOTTOM, INPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(PISTON, OUTPUT);
  //  attachInterrupt(digitalPinToInterrupt(EN1), encoder, RISING);
}

void loop() {
  if (RX_count == 1) {

    disconnected_counter = 0;

    Serial.println("Connected");
    ps_read();

    if (xj2 > (0.8 * pwm_range)) {
      comp++;
    }
    else if (xj2 < (0.8 * -pwm_range)) {
      comp--;
    }
    Serial.print("Comp is: ");
    Serial.println(comp);
    targetAngle = targetAngle + comp;
    comp = 0;
    int w = -gyro_pid(targetAngle, gyro_read());
    if (w > 127) {
      w = 127;
    }
    else if (w < -127) {
      w = -127;
    }
    drive(-yj1, xj1, w, gyro_read());


    if (yj2 > (0.8 * pwm_range)  &&  botMode == ROUND1) {
      RackFlag = 1;
    }
    else if (yj2 < (0.8 * -pwm_range)  &&  botMode == ROUND1) {
      RackFlag = 0;
    }

    if (RackFlag == 1) {
      rgb[0] = 0;
      rgb[1] = 50;
      rgb[2] = 200;
      goToPosition(1);
    }
    else {
      rgb[0] = 0;
      rgb[1] = 255;
      rgb[2] = 0;
      goToPosition(0);
    }

    if (botMode == 0 && RackFlag == 0) {
      rgb[0] = 0;
      rgb[1] = 255;
      rgb[2] = 0;
    }
    else if (botMode == 1) {
      rgb[0] = 140;
      rgb[1] = 0;
      rgb[2] = 200;
    }
  }


  else if (RX_count == 0) {
    disconnected_counter++;
    if (disconnected_counter > 10000) {
      Serial.println("Disconnected");
      rgb[0] = 255;
      rgb[1] = 0;
      rgb[2] = 0;
      drive(0, 0, 0, gyro_read());
    }
  }
  lights_write(rgb[0], rgb[1], rgb[2]);
}













//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- ROBOCLAW ---------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool roboclaw_setup() {
  roboclaw.begin(115200);
}

bool drive(int x, int y, int w, int theta) {
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(w);
  Serial.print("\t");
  Serial.print(theta);
  Serial.print("\t");

  float theta_radian = theta * DEG2RAD;
  //  Serial.print(theta);
  //  Serial.println(theta_radian);

  float temp = x * cos(theta_radian) - y * sin(theta_radian);
  y = x * sin(theta_radian) + y * cos(theta_radian);
  x = temp;

  Serial.print(x);
  Serial.print("\t");
  Serial.println(y);

  M1Speed = 0.33 * w + 0.58 * y + 0.33 * x;
  M2Speed = 0.33 * w - 0.58 * y + 0.33 * x;
  M3Speed = 0.33 * w - 0.67 * x;
  //  M1Speed = limitVar(M1Speed, 30000);
  //  M2Speed = limitVar(M2Speed, 30000);
  //  M3Speed = limitVar(M3Speed, 30000);
  if (M1Speed > 127) {
    M1Speed = 127;
  }
  else if (M1Speed < -127) {
    M1Speed = -127;
  }

  if (M2Speed > 127) {
    M2Speed = 127;
  }
  else if (M3Speed < -127) {
    M3Speed = -127;
  }

  if (M1Speed > 127) {
    M1Speed = 127;
  }
  else if (M1Speed < -127) {
    M1Speed = -127;
  }

  M1Speed = map(M1Speed, -127, 127, 0, 127);
  M2Speed = map(M2Speed, -127, 127, 0, 127);
  M3Speed = map(M3Speed, -127, 127, 0, 127);



  roboclaw.ForwardBackwardM1(address1, -M1Speed);
  roboclaw.ForwardBackwardM2(address1, M2Speed);
  roboclaw.ForwardBackwardM2(address2, -M3Speed);
  return true;
}

bool drive(int x, int y, int w) {
  M1Speed = 0.33 * w + 0.58 * y + 0.33 * x;
  M2Speed = 0.33 * w - 0.58 * y + 0.33 * x;
  M3Speed = 0.33 * w - 0.67 * x;
  //  M1Speed = limitVar(M1Speed, 30000);
  //  M2Speed = limitVar(M2Speed, 30000);
  //  M3Speed = limitVar(M3Speed, 30000);
  if (M1Speed > 127) {
    M1Speed = 127;
  }
  else if (M1Speed < -127) {
    M1Speed = -127;
  }

  if (M2Speed > 127) {
    M2Speed = 127;
  }
  else if (M3Speed < -127) {
    M3Speed = -127;
  }

  if (M1Speed > 127) {
    M1Speed = 127;
  }
  else if (M1Speed < -127) {
    M1Speed = -127;
  }

  M1Speed = map(M1Speed, -127, 127, 0, 127);
  M2Speed = map(M2Speed, -127, 127, 0, 127);
  M3Speed = map(M3Speed, -127, 127, 0, 127);



  roboclaw.ForwardBackwardM1(address1, -M1Speed);
  roboclaw.ForwardBackwardM2(address1, M2Speed);
  roboclaw.ForwardBackwardM2(address2, M3Speed);
  return true;
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
    if (botMode == ROUND2) {
      interUART_write('5');
      currentPos = 1;
    }
    else {
      RackFlag = 1;
    }
    Serial.println("Up");
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    if (botMode == ROUND2) {
      interUART_write('2');
      currentPos = 2;
    }
    else {
      RackFlag = 0;
    }
    butt[PS_DOWN] = 0;
  }

  if (butt[PS_LEFT] == 1)
  {
    if (botMode == ROUND2) {

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

    }
    else {
      digitalWrite(PISTON, LOW);
    }
    Serial.println("right");

    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1)
  {
    comp = comp + 10;
    Serial.println("square");
    butt[PS_SQUARE] = 0;
  }
  if (butt[PS_CIRCLE] == 1)
  {
    comp = comp - 10;
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

  if (butt[PS_R1] == 1)
  {
    if (botMode == ROUND2) {
      interUART_write('0');
      currentPos = 5;
    }
    else {
      digitalWrite(PISTON, !digitalRead(PISTON));
    }
    Serial.println("R1");
    butt[PS_R1] = 0;
  }

  if (butt[PS_L1] == 1)
  {
    if (botMode == ROUND1) {
      interUART_write('8'); //Deploy Stacking
      if (currentPos == 1) {
        //        interUART_write('0');
      }
      else if (currentPos == 5) {
        //        interUART_write('5');
      }
      botMode = ROUND2;
    }
    else {
      interUART_write('9'); //Close Stacking
      if (currentPos == 1) {
        //        interUART_write('0');
      }
      else if (currentPos == 5) {
        //        interUART_write('5');
      }
      botMode = ROUND1;
    }
    Serial.println("L1");
    butt[PS_L1] = 0;
  }

  if (butt[PS_PS] == 1)
  {
    if (botMode == ROUND2) {
      if (gripperState == 0) {
        gripperState = 1;
        interUART_write('6');
        if (currentPos == 1) {
          interUART_write('0');
        }
        else if (currentPos == 5) {
          interUART_write('5');
        }
      }
      else if (gripperState == 1) {
        gripperState = 0;
        interUART_write('7');
        if (currentPos == 1) {
          interUART_write('0');
        }
        else if (currentPos == 5) {
          interUART_write('5');
        }
      }
    }
    butt[PS_PS] = 0;
  }


  //  if (botMode == ROUND2) {
  //    if (currentPos == 0) {
  //      interUART_write('2');
  //    }
  //  }

  //  currentPos = 0;
}


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
//------------------------------------------------- BALL_PICKING-------------------------------------------
//---------------------------------------------------------------------------------------------------------
void goToPosition(int Pos) {
  if (Pos == 0) {
    //    Serial.println("Going Down");
    if (!digitalRead(BOTTOM)) {
      //      Serial.print("Value of sensor is: ");
      //      Serial.println(digitalRead(BOTTOM));
      //      Serial.println("Going Down");
      drive_rack(-255);
    }
    else {
      drive_rack(0);
      //      Serial.println("Stopped at Bottom");
    }
  }
  else if (Pos == 1) {
    //    Serial.println("Going Up");
    if (!digitalRead(TOP)) {
      //      Serial.print("Value of Sensor is: ");
      //      Serial.println(digitalRead(TOP));
      //      Serial.println("Going Up");
      drive_rack(255);
    }
    else {
      //      Serial.println("Stopped at Top");
      drive_rack(0);
    }
  }
}


void drive_rack(int vel) {
  if (vel > 0) {
    //    Serial.println(count);
    moveUpRack(vel);
  }
  else if (vel < 0) {
    vel = -vel;
    moveDownRack(vel);
    //    Serial.println(count);
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
int PID_rack(int currentValue, int targetValue) {
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error_rack;
  int diff_error = error - prev_error_rack;

  prev_error_rack = error;
  return (KP_rack * prop_error) + (KI_rack * integral_error) + (KD_rack * diff_error);
}

//void encoder() {
//  if (digitalRead(EN2)) {
//    count++;
//  }
//  else {
//    count--;
//  }
//}




//STACKING
void interUART_setup() {
  Serial2.begin(115200);
}

void interUART_write(char data) {
  Serial.println(data);
  Serial2.write(data);
}
