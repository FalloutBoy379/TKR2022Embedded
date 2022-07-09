//###################################################################################################################################################################
//############################################################ INCLUDES AND CLASS DEFINITIONS #######################################################################
//###################################################################################################################################################################

#include <MPU6050_light.h>
#include "Wire.h"

#include <RoboClaw.h>
#include <AutoPID.h>

RoboClaw roboclaw(&Serial1, 10000);

MPU6050 mpu(Wire);

#define BAUD 9600
#define BAUDRATE ((F_CPU / (BAUD * 16UL) - 1))

#define pwm_range 200

#define PS_L1 0
#define PS_R1 1
#define PS_L2 19
#define PS_R2 20
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
#define PS_MIC 2
#define PS_BUTTON 3

long Raftaar = 0;

double KPA = 157;
double KIA = 0;
double KDA = 0;

double KPG1 = 20;
double KIG1 = 5.7;
double KDG1 = 57;

double KPG = 10;
double KIG = 0.5;
double KDG = 42;

double KPP = 5.01;
double KIP = 3;
double KDP = 2;

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

int prevMillis = 0;
int xj1Offset = 0, yj1Offset = 0, xj2Offset = 0, yj2Offset = 0, calibrateFlag = 0, prevComp = 0;

int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0;  //analog values(serially received from remote);
int butt[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };               //digital values(serially received from remote);

uint8_t RX[16] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0;

uint8_t TX[16] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
int flag[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
double angz = 0;
double sudhaar = 0;

double zeroangle = 0;
double target = 0;
double targetangle = 0;

double potvalue = 0;
double correction = 0;
double targetpot = 840;
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//MOSFET 1,2  --> DUAL ACTUATING
//Defined pins for pistons
int centrePistonOut = 34;
int centrePistonIn = 30;
int leftPiston = 32;
int rightPiston = 28;
//Flags for pistons
int centrePiston_Flag = 0;
int leftPiston_Flag = 0;
int rightPiston_Flag = 0;
//Time for Actuations
long long time_centre = 0;
long long time_left = 0;
long long time_right = 0;

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

unsigned long long disconnectedcounter = 0;

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

AutoPID DRIVEPID(&angz, &targetangle, &sudhaar, -255.0 , 255.0, KPG, KIG, KDG);
AutoPID POTPID(&potvalue, &targetpot, &correction, -125.0 , 125.0, KPP, KIP, KDP);

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
class BTS {
  public:
    enum mode {
      FORWARD,
      REVERSED
    };

    BTS(int pwm, int dir1, int dir2) {
      pinMode(pwm, OUTPUT);
      pinMode(dir1, OUTPUT);
      pinMode(dir2, OUTPUT);
      PWM = pwm;
      DIR1 = dir1;
      DIR2 = dir2;
      setMaxPWM(255);
    }

    void setDirection(mode motormode) {
      if (motormode == REVERSED) {
        int temp = DIR1;
        DIR1 = DIR2;
        DIR2 = temp;
      }
    }

    void setMaxPWM(int maxVel) {
      MAXVEL = maxVel;
    }

    void runMotor(int vel) {
      if ((vel > 0 && vel <= 100) || (vel < 0 && vel >= -100 ))
      {
        vel = 0;
      }
      if (vel > MAXVEL) {
        vel = MAXVEL;
      }
      else if (vel < -MAXVEL) {
        vel = -MAXVEL;
      }

      if (vel >= 0) {
        runForward(vel);
      }
      else if (vel < 0) {
        runBackward(-vel);
      }
    }


  private:
    mode motorMode = FORWARD;
    int DIR1, DIR2, PWM, MAXVEL = 255;

    void runForward(int vel) {
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, LOW);
      analogWrite(PWM, vel);
    }

    void runBackward(int vel) {
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, HIGH);
      analogWrite(PWM, vel);
    }
};
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

#define M1PWM 46
#define M1DIR1 47
#define M1DIR2 48

#define M2PWM 44
#define M2DIR1 40
#define M2DIR2 42

#define M3PWM 6
#define M3DIR1 7
#define M3DIR2 8

#define PWPWM 4
#define PWDIR1 3
#define PWDIR2 5
//ed2 all 4 pins ed1 pin 4 ~ enable
#define POTPIN A1

BTS M1(M1PWM , M1DIR1 , M1DIR2), M2(M2PWM , M2DIR1 , M2DIR2), M3(M3PWM , M3DIR1 , M3DIR2), PW(PWPWM , PWDIR1 , PWDIR2);

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
uint8_t excessflag = 0;
uint8_t deficitflag = 0;
uint8_t flag1 = 0;
uint8_t flag2 = 0;
uint8_t Tstate = 0;
int count = 0;
int count1 = 0;
int ETFLAG = 0;
int ETFLAG1 = 0;
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
bool gyro_setup() {
  Wire.begin();

  mpu.setAddress(0x69);

  byte status = mpu.begin();

  while (status != 0) {
    Serial.println("FATALITY      -       Could not connect!!");
  }

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets();
  Serial.println("Done!\n");

  delay(1000);

  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
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
    if ((RX_raw > 230) && (RX_raw < 252))
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
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
bool ps_read()
{
  RX_count = 0;

  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, -pwm_range, pwm_range);
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, (-pwm_range), pwm_range);
  pot1 = map(RX[4], 0, RX_range, 0, (pwm_range));          //R2
  pot2 = map(RX[5], 0, RX_range, 0, (pwm_range));          //L2

  if (butt[PS_SHARE] == 1)
  {
    Serial.println("Share");
    target = 90;
    flag1 = 1;
    butt[PS_SHARE] = 0;
  }
  if (butt[PS_OPTIONS] == 1)
  {
    Serial.println("Options");
    target = 0;
    butt[PS_OPTIONS] = 0;
  }
  if (butt[PS_UP] == 1)
  {
    flag2 = 1;
    target = 65;
    targetpot = 818;
    Serial.println("Up");
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    flag2 = 1;
    target = -65;
    targetpot = 818;
    Serial.println("Down");
    butt[PS_DOWN] = 0;
  }
  if (butt[PS_LEFT] == 1)
  {
    target = 26;
    targetpot = 818;
    Serial.println("left");
    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    target = -26;
    targetpot = 818;
    Serial.println("right");
    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1)
  {
    leftPiston_Flag = 1;
    time_left = millis();
    Serial.println("square");
    butt[PS_SQUARE] = 0;
  }
  if (butt[PS_CIRCLE] == 1)
  {
    rightPiston_Flag = 1;
    time_right = millis();
    Serial.println("circle");
    butt[PS_CIRCLE] = 0;
  }
  if (butt[PS_TRIANGLE] == 1)
  {
    centrePiston_Flag = 1;
    time_centre = millis();
    Serial.println("triangle");
    butt[PS_TRIANGLE] = 0;
  }
  if (butt[PS_CROSS] == 1)
  {
    if (Tstate == 0)
    {
      Tstate = 7;
    }

    else if (Tstate == 1)
    {
      Tstate = 0;
    }

    Serial.println("cross");
    butt[PS_CROSS] = 0;
  }
  if (butt[PS_L1] == 1)
  {
    deficitflag = 1;
    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {
    excessflag = 1;
    Serial.println("R1");
    butt[PS_R1] = 0;
  }


  if (pot2 >= 180)              //L2
  {
    count = count + 1;
    if (count == 1 && ETFLAG == 0)
    {
      ETFLAG = 1;
      targetpot = targetpot + 20;
    }
    Serial.println("L2");
  }
  if (pot2 < 180)
  {
    ETFLAG = 0;
    count = 0;
  }


  if (pot1 >= 180)              //R2
  {
    count1 = count1 + 1;
    if (count1 == 1 && ETFLAG1 == 0)
    {
      ETFLAG1 = 1;
      targetpot = targetpot - 20;
    }
    Serial.println("R2");
  }
  if (pot1 < 180)
  {
    ETFLAG1 = 0;
    count1 = 0;
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
  if (butt[PS_MIC] == 1)
  {
    targetpot = 860;
    Serial.println("Mic");
    butt[PS_MIC] = 0;
  }
  if (butt[PS_BUTTON] == 1)
  {
    Serial.println("PS Button");
    butt[PS_BUTTON] = 0;
  }

}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void drive(double A_x, double A_y, double ang_s)
{
  //front left - BTS 2
  //front right - BTS 1
  //back - BTS 3

  long F1, F2 , F3;

  F1 = (0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F2 = -(0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F3 = (0 * A_x) - (0.666 * A_y) + (0.333 * ang_s);

  F1 = adjust(F1);
  F2 = adjust(F2);
  F3 = adjust(F3);

  M1.runMotor(F1);
  M2.runMotor(-F2);
  M3.runMotor(F3);

}

long adjust(long variable)
{
  variable = variable * 2;

  return (variable);

}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
double gyroread()
{
  mpu.update();
  int tangz = mpu.getAngleZ();
  return (tangz);
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void driveflags()
{
  if (excessflag == 1)//R1
  {
    DRIVEPID.setGains(KPA, KIA, KDA);
    zeroangle = zeroangle + 1;
    excessflag = 0;
  }
  if (deficitflag == 1)//L1
  {
    DRIVEPID.setGains(KPA, KIA, KDA);
    zeroangle = zeroangle - 1;
    deficitflag = 0;
  }
  if (flag1 == 1)//share
  {
    DRIVEPID.setGains(KPG, KIG, KDG);
    flag1 = 0;
  }
  if (flag2 == 1)//up down
  {
    DRIVEPID.setGains(KPG1, KIG1, KDG1);
    flag2 = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void Throwingflags()
{
  if (Tstate == 7)
  {
    Raftaar = 32000;
    Tstate = 1;
  }
  if (Tstate == 0)
  {
    while (Raftaar >= 0)
    {
      Raftaar = Raftaar - 5000;
      delay(70);
    }
    Raftaar = 0;
  }

}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void Hopperflags()
{
  if (leftPiston_Flag == 1)
  {
    digitalWrite(leftPiston, HIGH);
    if (millis() - time_left > 2000)
    {
      digitalWrite(leftPiston, LOW);
      leftPiston_Flag = 0;
    }
    if (leftPiston_Flag == 0)
    {
      digitalWrite(leftPiston, LOW);
    }
  }
  if (centrePiston_Flag == 1)
  {
    digitalWrite(centrePistonOut, HIGH);
    digitalWrite(centrePistonIn, LOW);
    if (millis() - time_centre > 69)
    {
      digitalWrite(centrePistonOut, LOW);
      digitalWrite(centrePistonIn, HIGH);
      centrePiston_Flag = 0;

    }
    if (centrePiston_Flag == 0)
    {
      digitalWrite(centrePistonOut, LOW);
      digitalWrite(centrePistonIn, HIGH);
    }

  }

  if (rightPiston_Flag == 1)
  {
    //Serial.println("SEXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    digitalWrite(rightPiston, HIGH);
    if (millis() - time_right > 2000)
    {
      digitalWrite(rightPiston, LOW);
      rightPiston_Flag = 0;
    }
    if (rightPiston_Flag == 0)
    {
      digitalWrite(rightPiston, LOW);
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void Throwing()
{
  roboclaw.DutyM2(129, -Raftaar);
  roboclaw.DutyM1(129, -Raftaar);
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void hopper_setup()
{

  pinMode(centrePistonOut, OUTPUT);
  pinMode(centrePistonIn, OUTPUT);
  pinMode(leftPiston, OUTPUT);
  pinMode(rightPiston, OUTPUT);

  digitalWrite(centrePistonOut, LOW);
  digitalWrite(centrePistonIn, HIGH);

  digitalWrite(leftPiston, LOW);
  digitalWrite(rightPiston, LOW);
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void PIDsetup()
{
  POTPID.setBangBang(500);
  POTPID.setTimeStep(0.1);

  DRIVEPID.setBangBang(100);
  DRIVEPID.setTimeStep(0.1);
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void powerwindow()
{
  PW.runMotor(-correction);
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void setup()
{

  roboclaw.begin(115200);
  Serial.begin(115200);

  Serial.println("Starting");

  gyro_setup();
  Serial.println("Gyro setup done");

  ps_setup();
  Serial.println("Ps setup done");

  hopper_setup();
  Serial.println("Hopper setup done");

  PIDsetup();
  Serial.println("PID setup done");
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
if(RX_count == 1)
  {
    disconnectedcounter = 0;
    
    targetangle = zeroangle + target;
    
    ps_read();
  
    driveflags();
  
    Throwingflags();
  
    Hopperflags();
  
    Throwing();
  
    potvalue = analogRead(POTPIN);
    POTPID.run();
  
    powerwindow();
  
    angz =  gyroread();
    DRIVEPID.run();
  
    drive(xj1, yj1, -sudhaar);
  }
  else
  {
    
    disconnectedcounter++;
    
    if (disconnectedcounter >= 100000)
    {
      drive(0, 0, 0);
      Raftaar = 0;
      Throwing();
      hopper_setup();

      leftPiston_Flag = 0;
      rightPiston_Flag = 0;
      centrePiston_Flag = 0;
    }
  }
}
