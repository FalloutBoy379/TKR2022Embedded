#include <RoboClaw.h>
#include "Wire.h"
#include <MPU6050_light.h>

RoboClaw roboclaw(&Serial1, 10000);

MPU6050 mpu(Wire);
unsigned long timer = 0;
int KP_Gyro = 7;
int KI_Gyro = 0;
int KD_Gyro = 0;

//Error variables
long int current_gyro_error = 0, prev_gyro_error = 0, p_gyro_error = 0, i_gyro_error = 0, d_gyro_error = 0, P_gyro = 0, I_gyro = 0, D_gyro = 0;
//Others
long int final_w = 0, compensation = 0, count_moving = 0, count_movingFlag = 0, pos = 0;

void gyroSetup();
long int getGyroData();
long int gyro_pid(int target, int current);

long int comp = 0;

float M1Speed, M2Speed, M3Speed;
#define address1 128
#define address2 129


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
int gyro_target = 0;

#define pwmrange 127

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

int upFlag, r1flag, l1flag;
void receive()
{
  RX_count = 0;
  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, pwm_range, (-pwm_range));
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));

  if (butt[PS_START] == 1)
  {
    Serial.println("start");
    butt[PS_START] = 0;
  }
  if (butt[PS_SELECT] == 1)
  {
    Serial.println("select");
    butt[PS_SELECT] = 0;
  }
  if (butt[PS_UP] == 1)
  {
    upFlag = 1;
    Serial.println("up");
    //    ThrowingFlag = 1;
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    //    ThrowingFlag = 0;
    Serial.println("down");
    butt[PS_DOWN] = 0;
  }
  if (butt[PS_LEFT] == 1)
  {
    Serial.println("left");
    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    Serial.println("right");
    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1)
  {
    Serial.println("square");
    comp--;
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
    //    calibrateFlag = 1;
    gyro_target = 0;
    Serial.println("triangle");
    butt[PS_TRIANGLE] = 0;
  }
  if (butt[PS_CROSS] == 1)
  {
    gyro_target = 90;
    //    angleval = 0;
    Serial.println("cross");
    butt[PS_CROSS] = 0;
  }
  if (butt[PS_L1] == 1)
  {
    if (upFlag == 1) {
      l1flag = 1;
    }
    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {
    if (l1flag == 1) {
      r1flag = 1;
    }
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
    Serial.println("L3");
    butt[PS_L3] = 0;
  }
  if (butt[PS_R3] == 1)
  {
    Serial.println("R3");
    butt[PS_R3] = 0;
  }
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


long int gyro_pid(int target, int current) {
  current_gyro_error = target - current;

  p_gyro_error = current_gyro_error;
  i_gyro_error = prev_gyro_error + current_gyro_error;
  d_gyro_error = current_gyro_error - prev_gyro_error;

  P_gyro = KP_Gyro * p_gyro_error;
  I_gyro = KI_Gyro * i_gyro_error;
  D_gyro = KD_Gyro * d_gyro_error;

  prev_gyro_error = current_gyro_error;
  return -(P_gyro + I_gyro + D_gyro);
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

float limitVar(float var, float target) {
  if (var >= target) {
    var = target;
  }
  else if (var <= -target) {
    var = -target;
  }

  return var;
}

void setup() {
  Serial.begin(115200);  // put your setup code here, to run once:
  serial3start();
  gyroSetup();
  roboclaw.begin(115200);
}

void loop() {
//  if (r1flag) {
    receive();
    //  xj1 = map(xj1, -127, 127, 0, pwmrange);
    //  yj1 = map(yj1, -127, 127, 0, pwmrange);
    Serial.print(xj1);
    Serial.print("\t");
    Serial.print(yj1);
    Serial.print("\t");
    Serial.print(mpu.getAngleZ());
    Serial.print("\t");
    mpu.update();
//    if(abs(gyro_target) > 90){
//      KP = 4;
//    }
    int w = gyro_pid(gyro_target, mpu.getAngleZ());
    final_w = w + (comp*10);

    if(final_w > 127){
      final_w = 127;
    }
    else if(final_w<-127){
      final_w = -127;
    }
    Serial.print(final_w);
    drive(-yj1, -xj1, final_w);
//  }
}
