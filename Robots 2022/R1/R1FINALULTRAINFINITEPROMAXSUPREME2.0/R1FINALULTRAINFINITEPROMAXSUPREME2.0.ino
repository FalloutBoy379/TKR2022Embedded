//#gyro with 2 motorss on
#include <RoboClaw.h>
#include <MPU6050_light.h>
#include "Wire.h"

MPU6050 mpu(Wire);

#define KPG 2.34
#define KIG 0.013
#define KDG 6.1

#define KPG1 5.1
#define KIG1 0.1
#define KDG1 20

#define KPG2 2.5
#define KIG2 0.1
#define KDG2 9.07

#define REDLED 9
#define GREENLED 11
#define BLUELED 10

#define back_BOHH 140
#define front_BOHH 117
#define lagori 79

double sudhaar = 0;
double angz = 0;

void lights_setup();

void lights_write(int r, int g, int b);

unsigned long int disconnectedcounter = 0;
long long a = 0;
RoboClaw roboclaw(&Serial1, 10000);

#define address1 128
#define address2 129

///////////////////////////////////////////////////////////////////////////////////////////

int induct1 = A2;
int induct2 = A3;
int induct3 = A4;
int induct4 = A5;
int induct5 = A6;
int induct6 = A7;

///////////////////////////////////////////////////////////////////////////////////////////
//Variables for POT PID

int potPin = A1;
int potVal = 0;
int PID_val = 0;
int PID_throw = 0;

int initialPotVal = 0;

///////////////////////////////////////////////////////////////////////////////////////////
//Defined pins for Throwing
int uPin = 44;
int uForward = 40;
int uBackward = 42;

int dPin = 46;
int dForward = 47;
int dBackward = 48;

volatile int buttonCount_throwing = 0;//button count for odd||even counts for starting or stopping Throwing
double targetangle = 0;
int duty = 0;
///////////////////////////////////////////////////////////////////////////////////////////
//Defined pins for pistons
int centrePistonOut = 32;
int centrePistonIn = 34;
int leftPiston = 28;
int rightPiston = 30;
//Flags for pistons
int centrePiston_Flag = 0;
int leftPiston_Flag = 0;
int rightPiston_Flag = 0;
//Time for Actuations
long long time_centre = 0;
long long time_left = 0;
long long time_right = 0;

///////////////////////////////////////////////////////////////////////////////////////////
//Flags for 4 BOHHs
int pelpendiculal_flag = 0;
int I_BOHH_Flag = 0;//-320
int A_BOHH_Flag = 0;//320
int K_BOHH_Flag = 0;//-640
int C_BOHH_Flag = 0;//640

int ppdflag = 0;// perpendicular flag -910
int zsflag = 0;// zero state flag 0

int Up_flag = 0;
int Down_flag = 0;
long long Up = 0;
long long Down = 0;

/////////////////////////////////////////////////////
int PID_drive(int currentValue_drive, int targetValue_drive);

//////////////////////////////////////////////////////////////////////////////////////////

#define KP_throwing 13                                         //PID Values for THROWING
#define KI_throwing 0.1
#define KD_throwing 2
int prev_error_throwing = 0;
int target_throwing = 0;

int PID_throwing(int currentValue_throwing, int targetValue_throwing);

///////////////////////////////////////////////////////////////////////////////////////////

#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))
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

int prevMillis = 0;
int xj1Offset = 0, yj1Offset = 0, xj2Offset = 0, yj2Offset = 0, calibrateFlag = 0, prevComp = 0;

int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0; //analog values(serially received from remote);
int butt[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //digital values(serially received from remote);

uint8_t RX[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0;
uint8_t TX[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
int flag[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;

int encresetflag = 0;


/////////////////////////////////////////////////////////////////////////////////////////////////

// Function ps_Setup() for PS
bool ps_setup() {
  UBRR3H = BAUDRATE >> 8;
  UBRR3L = BAUDRATE;
  UCSR3B = 0b10011000; //enable RXEN TXEN
  UCSR3C = 0b00000110;
}

ISR(USART3_RX_vect)
{
  RX_count = 1;
  disconnectedcounter = 0;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////
void lights_setup()
{
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
////////////////////////////////////////////////////////////////////////////////////////////////////////
void powerwindowDuty()                                                      //PowerWindow Throwing angle function
{
  //  roboclaw.DutyM1(address1, duty);
  roboclaw.DutyM1(address1, PID_throw);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to read data from PS : ps_read()
//READ DATA FROM CONTROLLER
bool ps_read() {
  RX_count = 0;
  disconnectedcounter++;
  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, -pwm_range, (pwm_range));
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));
  pot1 = map(RX[4], 0, RX_range, 0, (pwm_range));          //R2
  pot2 = map(RX[5], 0, RX_range, 0, (pwm_range));          //L2

  if (butt[PS_SHARE] == 1)
  {
    Serial.println("Share");
    ppdflag = 1;
    butt[PS_SHARE] = 0;
  }
  if (butt[PS_OPTIONS] == 1)
  {
    Serial.println("Options");
    zsflag = 1;
    butt[PS_OPTIONS] = 0;
  }
  if (butt[PS_UP] == 1)
  {
    Up_flag = 1;
    Up = millis();
    Serial.println("Up");
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    Down_flag = 1;
    Down = millis();
    Serial.println("Down");
    butt[PS_DOWN] = 0;
  }
  if (butt[PS_LEFT] == 1)
  {
    I_BOHH_Flag = 1;
    Serial.println("left");
    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    A_BOHH_Flag = 1;
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
    buttonCount_throwing = buttonCount_throwing + 1 ;
    Serial.println("cross");
    butt[PS_CROSS] = 0;

    a = millis();
  }
  if (butt[PS_L1] == 1)
  {
    C_BOHH_Flag = 1;
    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {
    K_BOHH_Flag = 1;
    Serial.println("R1");
    butt[PS_R1] = 0;
  }
  if (pot2 >= 180)              //L2
  {
    Serial.println("L2");
  }
  if (pot1 >= 180)              //R2
  {
    Serial.println("R2");
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
    pelpendiculal_flag = 1;
    Serial.println("Mic");
    butt[PS_MIC] = 0;
  }
  if (butt[PS_BUTTON] == 1)
  {
    Serial.println("PS Button");
    butt[PS_BUTTON] = 0;
  }

}
/////////////////////////////////////////////////////////////////////////////////////////////

void drive(double A_x, double A_y, double ang_s)
{
  double F1, F2 ,F3;

  F1 = (0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F2 = -(0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F3 = (0*A_x) - (0.666*A_y) + (0.333*ang_s);

  F1 = F1 * 200;
  F2 = F2 * 200;
  F3 = F3 * 200;

  if (F1 > 25000)
  {
    F1 = 25000;
  }
  if (F2 > 25000)
  {
    F2 = 25000;
  }
  if (F1 < -25000)
  {
    F1 = -25000;
  }
  if (F2 < -25000)
  {
    F2 = -25000;
  }
  
  if (F3 < -25000)
  {
    F3 = -25000;
  }
  if (F3 < -25000)
  {
    F3 = -25000;
  }


  Serial.print(F1);
  Serial.print("\t");
  Serial.print(F2);
  Serial.print("\t");
  Serial.print(F3);
  Serial.print("\n");

  roboclaw.DutyM2(128, -F1);
  roboclaw.DutyM2(129, F2);
  roboclaw.DutyM1(129, -F3);
  



}
/////////////////////////////////////////////////////////////////////////////////////////////

double error_drive = 0;
double integral_error_drive = 0;
double prev_error_drive = 0;

double PID_drive(double currentValue_drive, double targetValue_drive)                //PID code block for DRIVE HARDCODED
{
  error_drive = targetValue_drive - currentValue_drive;

  double prop_error_drive = error_drive;
  integral_error_drive = error_drive + prev_error_drive;
  double diff_error_drive = error_drive - prev_error_drive;

  prev_error_drive = error_drive;
  return ((KPG * prop_error_drive) + (KIG * integral_error_drive) + (KDG * diff_error_drive));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
double error_drive1 = 0;
double integral_error_drive1 = 0;
double prev_error_drive1 = 0;

double PID_drive1(double currentValue_drive1, double targetValue_drive1)                //PID code block for DRIVE HARDCODED
{
  error_drive1 = targetValue_drive1 - currentValue_drive1;

  double prop_error_drive1 = error_drive1;
  integral_error_drive1 = error_drive1 + prev_error_drive1;
  double diff_error_drive1 = error_drive1 - prev_error_drive1;

  prev_error_drive1 = error_drive1;
  return ((KPG1 * prop_error_drive1) + (KIG1 * integral_error_drive1) + (KDG1 * diff_error_drive1));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
double error_drive2 = 0;
double integral_error_drive2 = 0;
double prev_error_drive2 = 0;

double PID_drive2(double currentValue_drive2, double targetValue_drive2)                //PID code block for DRIVE HARDCODED
{
  error_drive2 = targetValue_drive2 - currentValue_drive2;

  double prop_error_drive2 = error_drive2;
  integral_error_drive2 = error_drive2 + prev_error_drive2;
  double diff_error_drive2 = error_drive2 - prev_error_drive2;

  prev_error_drive2 = error_drive2;
  return ((KPG2 * prop_error_drive2) + (KIG2 * integral_error_drive2) + (KDG2 * diff_error_drive2));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
int integral_error_throwing = 0;

int PID_throwing(int currentValue_throwing, int targetValue_throwing)      //PID code block for THROWING
{
  int error_throwing = targetValue_throwing - currentValue_throwing;

  int prop_error_throwing = error_throwing;
  integral_error_throwing = integral_error_throwing + prev_error_throwing;
  int diff_error_throwing = error_throwing - prev_error_throwing;

  prev_error_throwing = error_throwing;
  return ((KP_throwing * prop_error_throwing) + (KI_throwing * integral_error_throwing) + (KD_throwing * diff_error_throwing));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function for Throwing
int i = 0;
int j = 0;
int k = 255;
int l = 255;
void Throwing(int a, int b)
{
  digitalWrite(uForward, HIGH);
  digitalWrite(uBackward, LOW);
  analogWrite(uPin, a);

  digitalWrite(dBackward, HIGH);
  digitalWrite(dForward, LOW);
  analogWrite(dPin, b);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  ps_setup();
  roboclaw.begin(115200);
  Serial.begin(115200);


  Wire.begin();
  pinMode(centrePistonOut, OUTPUT);
  pinMode(centrePistonIn, OUTPUT);
  pinMode(leftPiston, OUTPUT);
  pinMode(rightPiston, OUTPUT);


  mpu.setAddress(0x69);
  byte status = mpu.begin();
  Serial.print(F("MPU6050-1 status: "));
  Serial.println(status);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  pinMode(uPin, OUTPUT);
  pinMode(dPin, OUTPUT);
  pinMode(uForward, OUTPUT);
  pinMode(uBackward, OUTPUT);
  pinMode(dForward, OUTPUT);
  pinMode(dBackward, OUTPUT);

  digitalWrite(centrePistonOut, LOW);
  digitalWrite(centrePistonIn, HIGH);

  digitalWrite(leftPiston, LOW);
  digitalWrite(rightPiston, LOW);

  lights_setup();

  lights_write(255, 0, 0);

  while(digitalRead(induct1) == LOW){
   roboclaw.DutyM1(address1, 8000);
  }
  delay(200);
  initialPotVal = analogRead(potPin);
  target_throwing = initialPotVal;
}

int shifter = 0;
int r = 0;
int g = 255;
int b = 0;

int manualflag = 0;

void loop()                                                                                              // Code which has to be on loop
{
  if (RX_count == 1)
  {
    ps_read();

    mpu.update();
    //    r = r + 30;
    //    g = g - 20;
    //    b = b + 69;
    //
    //    if (r >= 255)
    //    {
    //      r = 0;
    //    }
    //
    //    if (b >= 255)
    //    {
    //      b = 0;
    //    }
    //
    //    if (g <= 0)
    //    {
    //      g = 255;
    //    }
   
    lights_write(0, 255, 0);
    if(xj2>=100)
    {
      manualflag = 1;
      targetangle = targetangle + 0.25;
    }
        if(xj2<=-100)
    {
      manualflag = 1;
      targetangle = targetangle - 0.25;
    }
    if (Up_flag == 1)
    {
      manualflag = 0;
      target_throwing = initialPotVal + lagori;
//      duty = -20000;
      powerwindowDuty();
//      if (millis() - Up > 100)
//      {
//
//        duty = 0;
//        powerwindowDuty();
//        Up_flag = 0;
//      }
      Up_flag = 0;
    }
    if (Down_flag == 1)
    {
      manualflag = 0;
//      duty = 20000;
      powerwindowDuty();
//      if (millis() - Down > 100)
//      {
//        duty = 0;
//        powerwindowDuty();
//        Down_flag = 0;
//      }
      Down_flag = 0;
    }

    /*____________________________________________________________________________________________________*/
    //Flag conditions for BOHH targets

    if (ppdflag == 1)//share
    {
      manualflag = 0;
      shifter = 0;
      targetangle = 90;
      ppdflag = 0;
    }

    if (zsflag == 1)//select
    {
      manualflag = 0;
      targetangle = 0;
      zsflag = 0;
    }

    if (I_BOHH_Flag == 1)//left
    {
      manualflag = 0;
      shifter = 1;
      targetangle = 25;
      target_throwing = initialPotVal + back_BOHH;
      I_BOHH_Flag = 0;
    }
    if (A_BOHH_Flag == 1)//right
    {
      manualflag = 0;
      shifter = 1;
      targetangle = -25;
      target_throwing = initialPotVal + back_BOHH;
      A_BOHH_Flag = 0;
    }

    if (K_BOHH_Flag == 1)//R1
    {
      manualflag = 0;
      shifter = 2;
      targetangle = 66;
      target_throwing = initialPotVal + front_BOHH;
      K_BOHH_Flag = 0;
    }
    if (C_BOHH_Flag == 1)//L1
    {
      manualflag = 0;
      shifter = 2;
      targetangle = -66;
      target_throwing = initialPotVal + front_BOHH;
      C_BOHH_Flag = 0;
    }
   /*____________________________________________________________________________________________________*/
    // Throwing PID

    PID_val = - PID_throwing(analogRead(potPin), target_throwing);
    PID_throw = map(PID_val,-10000,10000,-32000,32000);
    if(PID_throw>0 && PID_throw<1000){
      PID_throw = 0;
    }
    else if(PID_throw<0 && PID_throw>-1000){
      PID_throw = 0;
    }
    else if(PID_throw>1000 && PID_throw<7000){
      PID_throw = 7000;
    }
    else if(PID_throw<-1000 && PID_throw>-7000){
      PID_throw = -7000;
    }
    else if(PID_throw>10000){
      PID_throw = 15000;
    }
    else if(PID_throw<-10000){
      PID_throw = -15000;
    }
//    PID_throw = PID_val;

    powerwindowDuty();    
    /*____________________________________________________________________________________________________*/
    //Centre Piston Actuation
    if (centrePiston_Flag == 1)
    {
      manualflag = 0;
      digitalWrite(centrePistonOut, HIGH);
      digitalWrite(centrePistonIn, LOW);
      if (millis() - time_centre > 200)
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
    /*____________________________________________________________________________________________________*/
    //Left Piston Actuation
    if (leftPiston_Flag == 1)
    {
      manualflag = 0;
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
    /*____________________________________________________________________________________________________*/
    //Right Piston Actuation
    if (rightPiston_Flag == 1)
    {
      manualflag = 0;
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
    /*____________________________________________________________________________________________________*/
    //Throwing
    if (buttonCount_throwing % 2 != 0)
    {
      manualflag = 0;
      k = 250;
      l = 200;

      Throwing(k, l);

    }
    if (buttonCount_throwing % 2 == 0 && buttonCount_throwing != 0)
    {
      manualflag = 0;
      k = 0;
      l = 0;
      Throwing(k, l);

      i = 0;
      j = 0;
    }


    if (shifter == 0)
    {
      sudhaar = PID_drive(mpu.getAngleZ(), targetangle);
    }
    if (shifter == 1 || manualflag == 1)
    {
      sudhaar = PID_drive1(mpu.getAngleZ(), targetangle);
    }
    if (shifter == 2)
    {
      sudhaar = PID_drive2(mpu.getAngleZ(), targetangle);
    }

    drive(-xj1, -yj1, sudhaar);

  }
  else
  {
    lights_write(255, 0, 0);

    if (disconnectedcounter >= 100000)
    {
      drive(0, 0, 0);
      Throwing(0, 0);

      int pelpendiculal_flag = 0;
      int I_BOHH_Flag = 0;
      int A_BOHH_Flag = 0;
      int K_BOHH_Flag = 0;
      int C_BOHH_Flag = 0;


      int centrePiston_Flag = 0;
      int leftPiston_Flag = 0;
      int rightPiston_Flag = 0;
    }
  }
}
