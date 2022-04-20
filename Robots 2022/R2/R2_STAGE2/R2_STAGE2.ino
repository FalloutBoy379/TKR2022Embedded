#include <RoboClaw.h>
#include <Wire.h>
#include <MPU6050_light.h>


//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LIGHTS -------------------------------------------------
//---------------------------------------------------------------------------------------------------------
#define REDLED 13
#define GREENLED 11
#define BLUELED 12

void lights_setup();
void lights_write(int r, int g, int b);

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

#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))
#define pwm_range 115
#define PS_L1 0
#define PS_R1 1
#define PS_L2 2
#define PS_R2 3
#define PS_L3 4
#define PS_R3 5
#define PS_TRIANGLE 6
#define PS_SQUARE 7
#define PS_CROSS 8
#define PS_CIRCLE 9
#define PS_UP 10
#define PS_LEFT 11
#define PS_DOWN 12
#define PS_RIGHT 13
#define PS_START 14
#define PS_SELECT 15

int prevMillis = 0;
int xj1Offset = 0, yj1Offset = 0, xj2Offset = 0, yj2Offset = 0, calibrateFlag = 0, prevComp = 0;

int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0; //analog values(serially received from remote);
int butt[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //digital values(serially received from remote);

uint8_t RX[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0;
uint8_t TX[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
int flag[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;

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

void interUART_setup();
void interUART_write(char data);

//---------------------------------------------------------------------------------------------------------
//--------------------------------------------------  SETUP & LOOP  ---------------------------------------
//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);   // Intialize terminal serial port
  Serial.println("Starting...");
  lights_setup();
  roboclaw_setup();
  gyro_setup();
  ps_setup();
  interUART_setup();
  delay(1000);
  interUART_write('7'); //Close Gripper
  interUART_write('8'); //Deploy Stacking
  interUART_write('5'); //Home Stacking  
}
void loop() {
  if (RX_count == 1) {
    lights_write(0, 255, 0);
    ps_read();
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
    Serial.print(comp);
    Serial.print("\t");
    Serial.print(targetAngle);
    Serial.print("\t");
    Serial.print(gyro_read());
    Serial.print("\t");
    Serial.println(w);
    drive(yj1, xj1, w);
  }
}

//-------------------------------------------------- DEFINITIONS ------------------------------------------

void interUART_setup() {
  Serial2.begin(115200);
}

void interUART_write(char data) {
  Serial.println(data);
  Serial2.write(data);
}

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

bool roboclaw_setup() {
  roboclaw.begin(115200);
}

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

  if (butt[PS_START] == 1)
  {
    Serial.println("Start");
    butt[PS_START] = 0;
  }
  if (butt[PS_SELECT] == 1)
  {
    Serial.println("Select");
    butt[PS_SELECT] = 0;
  }
  if (butt[PS_UP] == 1)
  {
    interUART_write('1');
    interUART_write('2');
    interUART_write('3');
    interUART_write('4');
    Serial.println("Up");
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    interUART_write('2');
    Serial.println("Down");
    butt[PS_DOWN] = 0;
  }
  if (butt[PS_LEFT] == 1)
  {
    interUART_write('3');
    Serial.println("left");
    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    interUART_write('1');
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
    targetAngle = 0;
    loopVar = 3;
    loop3Counter = millis();
    Serial.println("triangle");
    butt[PS_TRIANGLE] = 0;
  }
  if (butt[PS_CROSS] == 1)
  {
    targetAngle = 90;
    Serial.println("cross");
    butt[PS_CROSS] = 0;
  }
  if (butt[PS_L1] == 1)
  {
    interUART_write('0');
    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {
   
    Serial.println("R1");
    butt[PS_R1] = 0;
  }
  if (butt[PS_L2] == 1)
  {
    butt[PS_L2] = 0;
  }
  if (butt[PS_R2] == 1)
  {
    butt[PS_R2] = 0;
  }
  if (butt[PS_L3] == 1)
  {
    loopVar = 2;
    Serial.println("L3");
    butt[PS_L3] = 0;
  }
  if (butt[PS_R3] == 1)
  {
    Serial.println("R3");
    butt[PS_R3] = 0;
  }
}

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
  Serial.print("\t");
  Serial.print(M1Speed);
  Serial.print("\t");
  Serial.print(-M2Speed);
  Serial.print("\t");
  Serial.println(-M3Speed);
  roboclaw.ForwardBackwardM1(address1, M1Speed);
  roboclaw.ForwardBackwardM2(address1, -M2Speed);
  roboclaw.ForwardBackwardM2(address2, -M3Speed);
  return true;
}
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

float limitVar(float var, float target) {
  if (var >= target) {
    var = target;
  }
  else if (var <= -target) {
    var = -target;
  }
  return var;
}
