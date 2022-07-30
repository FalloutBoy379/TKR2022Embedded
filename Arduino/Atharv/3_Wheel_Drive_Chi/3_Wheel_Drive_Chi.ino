#include <RoboClaw.h>
#include "Wire.h"
#include <MPU6050_light.h>


//DRIVE
/*****/
void drive(double A_x, double A_y, double ang_s);
double vel_x = 00, vel_y = 00, vel_ang = 00;
/*****/

//ROBOCLAW
/*****/
RoboClaw roboclaw(&Serial1, 10000);      //Pass Serial1 by reference to Roboclaw constructor
#define address1 128                     //Address for 1st Roboclaw
#define address2 129                     //Address for 2nd Roboclaw
/*****/

//GYRO
/*****/
MPU6050 mpu(Wire);
void gyro_init();//initialises gyro
double gyro_val();//gives gyro value
double bot_yaw, error1; //yaw value of drive
void gyro_pid();//pid function for yaw correction.Takes in error(in this case, the value from gyro_val() and returns the speed for gyro correction)
double yaw_correct;//value returned from gyro_pid is stored in this and this is given to drive function
unsigned long gyro_timer = 0;
/*****/

//PS
/*****/

#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))

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

int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0; //analog values(serially received from remote);
int butt[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //digital values(serially received from remote);

uint8_t RX[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0, pwm_range = 100;
uint8_t TX[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
int flag[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;
void serial3start();
void receieve();
double extra = 0, w = 0;
/*****/

//THROWING
/***/
int M1A = 40;
int M1B = 41;
int M1P = 5;
int M2A = 43;
int M2B = 44;
int M2P = 6;

int speed_throwing;
int throwing_speed = 100, lastTime = 0;
/***/
void throwing_init();
void throw_ball();
void change_throwing_angle(int );
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  roboclaw.begin(115200);//for roboclaw init
  gyro_init();
  serial3start();
  throwing_init();
}

void loop()
{

  // put your main code here, to run repeatedly:
  receieve();
  bot_yaw = gyro_val();
  gyro_pid();
  Serial.println(yj2);
  drive(00, 0, yaw_correct); //from trial and error, yaw_correct is reversed
  //throw_ball();
  change_throwing_angle(yj2);
  speed_throwing = 0;
}

void drive(double A_x, double A_y, double ang_s)
{
  int F1, F2, F3;
  /*
     F3 is head and is connected to M2 of BEAST
  */
  F1 = (0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F2 = -(0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F3 = (0 * A_x) - (0.666 * A_y) + (0.333 * ang_s);

  F1 = map(F1, -127, 127, -32000, 32000);
  F2 = map(F2, -127, 127, -32000, 32000);
  F3 = map(F3, -127, 127, -32000, 32000);
  roboclaw.DutyM1(address1, -F1);
  roboclaw.DutyM2(address1, F3);
  roboclaw.DutyM2(address2, F2);
}

void gyro_init()
{
  Wire.begin();

  byte status = mpu.begin();
  while (status != 0) { } // stop everything if could not connect to MPU6050

  //Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
}

double gyro_val()
{
  mpu.update();
  double bot_yaw1;
  if ((millis() - gyro_timer) > 10)
  { // print data every 10ms
    bot_yaw1 = mpu.getAngleZ();
    //Serial.println(bot_yaw1);
    gyro_timer = millis();
  }
  return bot_yaw1;
}

void gyro_pid()
{
  /*
     currently(25-01) with P , it avoids drift  while in motion.But,doesn't feel corrected .Not tested for rotating to given angle.D would be involved in such case
  */
  double P, I, D;
  double k_d_gyro = 24, k_p_gyro = 12 , k_i_gyro = 0.000;
  double error1_prev, correct_gyro;

  error1 = bot_yaw;
  error1  = error1 + extra;
  P = k_p_gyro * error1;
  I = k_i_gyro * error1 + I;
  D = k_d_gyro * (error1 - error1_prev);

  if (abs(error1) < 2)I = 0;
  error1_prev = error1;

  correct_gyro = P + D + I;
  yaw_correct =  -correct_gyro;
}


void serial3start()
{
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
    //    if ((RX_raw>210) && (RX_raw<227))
    //    {
    //      uint8_t r_temp0=(RX_raw-211);
    //      butt_rel[r_temp0]=1;
    //    }
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

void receieve()
{
  RX_count = 0;
  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, pwm_range, (-pwm_range));
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));

  //yj1 = RX[0];
  //xj1 = RX[1];
  //yj2 = RX[2];
  //xj2 = RX[3];

  if (butt[PS_START] == 1)
  {
    butt[PS_START] = 0;
  }
  if (butt[PS_SELECT] == 1)
  {
    butt[PS_SELECT] = 0;
  }
  if (butt[PS_UP] == 1)
  {

    extra++;
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    extra--;
    butt[PS_DOWN] = 0;
  }
  if (butt[PS_LEFT] == 1)
  {
    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1)
  {
    butt[PS_SQUARE] = 0;
  }
  if (butt[PS_CIRCLE] == 1)
  {
    butt[PS_CIRCLE] = 0;
  }
  if (butt[PS_TRIANGLE] == 1)
  {
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, LOW);
    analogWrite(M1P, 0);
    analogWrite(M2P, 0);
    Serial.println("OFF");
    butt[PS_TRIANGLE] = 0;
  }
  if (butt[PS_CROSS] == 1)
  {
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, HIGH);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, HIGH);
    analogWrite(M1P, 255);
    analogWrite(M2P, 255);
    Serial.println("ON");
    butt[PS_CROSS] = 0;
  }
  if (butt[PS_L1] == 1)
  {
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {
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
    butt[PS_L3] = 0;
  }
  if (butt[PS_R3] == 1)
  {
    butt[PS_R3] = 0;
  }
}

void throwing_init() {
  pinMode(M1A, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2P, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M1A, OUTPUT);
  pinMode(M1P, OUTPUT);
}

void change_throwing_angle(int speed_angle) {
  speed_angle = map(speed_angle, -127, 127, -30000, 30000);
  roboclaw.DutyM1(address2, speed_angle);
}

void throw_ball() {
  if (millis() - lastTime > 1000) {
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, LOW);
    analogWrite(M1P, 0);
    analogWrite(M2P, 0);
  }
  else {
    digitalWrite(M1A, HIGH);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, HIGH);
    digitalWrite(M2B, LOW);
    analogWrite(M1P, speed_throwing);
    analogWrite(M2P, speed_throwing);
  }
  lastTime = millis();
}
