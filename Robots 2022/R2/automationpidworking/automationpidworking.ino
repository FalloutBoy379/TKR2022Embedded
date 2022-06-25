#include <RoboClaw.h>
#include <Wire.h>
#include <MPU6050_light.h>

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LASER -------------------------------------------------
//---------------------------------------------------------------------------------------------------------
#define xlaser A1
#define xanalog A2
#define ylaser A3
#define yanalog A4

int x_pid(int xtarget, int xcurrent);
int y_pid(int ytarget, int ycurrent);

int xfilter(int xfinal);
int yfilter(int yfinal);

#define xKP 7 
#define xKI 0
#define xKD 7
int prevxerror=0;

#define yKP 7
#define yKI 0
#define yKD 7
int prevyerror=0;

int xprev =0;
int prevtx = 0;
int curtx;
float xfinal;
float dlpfx;

int yprev =0;
int prevty = 0;
int curty;
float yfinal;
float dlpfy;

int xsum = 0;
int ysum = 0;
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

void setup() 
{
  Serial.begin(115200);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  roboclaw_setup();
  gyro_setup();
}

void loop() 
{
  targetAngle = targetAngle + comp * 10;
  comp = 0;
  int w = -gyro_pid(targetAngle, gyro_read());
  w = w;
  if (w > 127) 
  {
    w = 127;
  }
  else if (w < -127) 
  {
    w = -127;
  }

  xsum = xfilter(analogRead(xlaser));
  ysum = yfilter(analogRead(ylaser));
  
  int xj1 = x_pid(500, xsum);
  xj1 = xj1;
  if (xj1 > 127) 
  {
    xj1 = 127;
  }
  else if (xj1 < -127) 
  {
    xj1 = -127;
  }
  
  int yj1 = y_pid(350, ysum);
  yj1 = yj1;
  if (yj1 > 127) 
  {
    yj1 = 127;
  }
  else if (yj1 < -127) 
  {
    yj1 = -127;
  }
  
  drive(xj1, yj1, -w);
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
  
  M1Speed = map(M1Speed, -127, 127, 10, 117);
  M2Speed = map(M2Speed, -127, 127, 10, 117);
  M3Speed = map(M3Speed, -127, 127, 10, 117);
  roboclaw.ForwardBackwardM1(address1, -M1Speed);
  roboclaw.ForwardBackwardM2(address2, -M2Speed);
  roboclaw.ForwardBackwardM2(address1, -M3Speed);
  return true;
}

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LASER -------------------------------------------------
//---------------------------------------------------------------------------------------------------------
int x_pid(int xtarget , int xcurrent)
{
  int error = xtarget - xcurrent;
  int prop_error = error;
  int integral_error = error + prevxerror;
  int diff_error = error - prevxerror;
  prevxerror = error;
  return (xKP*prop_error)+(xKI*integral_error)+(xKD*diff_error);
}

int y_pid(int ytarget , int ycurrent)
{
  int error = ytarget - ycurrent;
  int prop_error = error;
  int integral_error = error + prevyerror;
  int diff_error = error - prevyerror;

  prevyerror = error;
  return (yKP*prop_error)+(yKI*integral_error)+(yKD*diff_error);
}

int xfilter(int xfinal)
{
  curtx = millis();
  //xfinal = analogRead(xlaser);
  dlpfx = (xfinal-xprev)/(curtx - prevtx);
  prevtx = millis();
  if (dlpfx > 0.005)
  {
    xfinal = xprev;
  }
  xprev = xfinal;
  return xfinal;
}

int yfilter(int yfinal)
{
  curty = millis();
  //yfinal = analogRead(ylaser);
  dlpfy = (yfinal-yprev)/(curty - prevty);
  prevty = millis();
  if (dlpfy > 0.005)
  {
    yfinal = yprev;
  }
  yprev = yfinal;
  return yfinal;
}
