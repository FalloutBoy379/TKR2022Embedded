#include <RoboClaw.h>
#include "Wire.h"
#include <MPU6050_light.h>

RoboClaw roboclaw(&Serial1, 10000);
#define address1 128
#define address2 129

MPU6050 mpu(Wire);
MPU6050 mpu2(Wire);

///////////////////////////////////////////////////////////////////////////////////////////
                                                          //Pins for Mosfet switching of Gyro
int gyroSwitch1 = 5;
int gyroSwitch2 = 6;

///////////////////////////////////////////////////////////////////////////////////////////
                                                          //Defined pins for Throwing
int uPin = 9;
int uForward = 40;
int uBackward = 41;

int dPin = 10;
int dForward = 42;
int dBackward = 43;

volatile int buttonCount_throwing = 0;                             //button count for odd||even counts for starting or stopping Throwing

///////////////////////////////////////////////////////////////////////////////////////////
                                                          //Defined pins for pistons
int centrePistonOut = 32;
int centrePistonIn = 30;
int leftPiston = 26;
int rightPiston = 28;
                                                          //Flags for pistons
int centrePiston_Flag = 0;
int leftPiston_Flag = 0;
int rightPiston_Flag = 0;
                                                          //Time for Actuations
int time_centre = 0;
int time_left = 0;
int time_right = 0;

///////////////////////////////////////////////////////////////////////////////////////////
                                                          //Target variables for PID
int targetGyroAngle_drive = 0;
int targetGyroAngle_throwing = 0;

///////////////////////////////////////////////////////////////////////////////////////////
                                                          //Flags for 11 BOHHs
int I_BOHH_Flag = 0;
int A_BOHH_Flag = 0;
int K_BOHH_Flag = 0;
int C_BOHH_Flag = 0;
int H_BOHH_Flag = 0;
int E_BOHH_Flag = 0;
int J_BOHH_Flag = 0;
int B_BOHH_Flag = 0;
int G_BOHH_Flag = 0;
int D_BOHH_Flag = 0;
int F_BOHH_Flag = 0;

///////////////////////////////////////////////////////////////////////////////////////////

#define KP_drive 0                                         //PID Values for DRIVE HARDCODED
#define KI_drive 0
#define KD_drive 0
int prev_error_drive = 0;

int PID_drive(int currentValue_drive, int targetValue_drive);

///////////////////////////////////////////////////////////////////////////////////////////

//#define KP_driveIP 0                                       //PID Values for DRIVE with IP
//#define KI_driveIP 0
//#define KD_driveIP 0
//int prev_error_driveIP = 0;
//
//int PID_driveIP(int currentValue_driveIP, int targetValue_driveIP);

//////////////////////////////////////////////////////////////////////////////////////////

#define KP_throwing 0                                         //PID Values for THROWING
#define KI_throwing 0
#define KD_throwing 0
int prev_error_throwing = 0;

int PID_throwing(int currentValue_throwing, int targetValue_throwing);

///////////////////////////////////////////////////////////////////////////////////////////

// Standard PS Code
//Call ps_setup() in void setup()
//Call ps_read() in void loop()
                                                                       // Values defined for PS Code Block
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
#define PS_TOUCHPAD 18
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

                                                                                 // Function to read data from PS : ps_read()
//READ DATA FROM CONTROLLER
bool ps_read() {
  RX_count = 0;
  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, pwm_range, (-pwm_range));
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));
  pot1 = map(RX[4], 0, RX_range, 0,(pwm_range));           //R2
  pot2 = map(RX[5], 0, RX_range, 0,(pwm_range));           //L2

  if (butt[PS_SHARE] == 1)
  {
    H_BOHH_Flag = 1;
    Serial.println("Share");
    butt[PS_SHARE] = 0;
  }
  if (butt[PS_OPTIONS] == 1)
  {
    E_BOHH_Flag = 1;
    Serial.println("Options");
    butt[PS_OPTIONS] = 0;
  }
  if (butt[PS_UP] == 1)
  {
    K_BOHH_Flag = 1;
    Serial.println("Up");
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    C_BOHH_Flag = 1;
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
  }
  if (butt[PS_L1] == 1)
  {
    J_BOHH_Flag = 1;
    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {
    B_BOHH_Flag = 1;
    Serial.println("R1");
    butt[PS_R1] = 0;
  }
  if (pot2 >= 180)              //L2
  {
    G_BOHH_Flag = 1;
    Serial.println("L2");
  }
  if (pot1 >= 180)              //R2
  {
    D_BOHH_Flag = 1;
    Serial.println("R2");
  }
  if (butt[PS_L3] == 1)
  {
    targetGyroAngle_throwing = 0;
    Serial.println("L3");
    butt[PS_L3] = 0;
  }
  if (butt[PS_R3] == 1)
  {
    targetGyroAngle_drive = 90;
    Serial.println("R3");
    butt[PS_R3] = 0;
  }
  if (butt[PS_MIC] == 1)
  {
    gyroSetup;
    Serial.println("Mic");
    butt[PS_MIC] = 0;
  }
  if (butt[PS_TOUCHPAD] == 1)
  {
    F_BOHH_Flag = 1;
    Serial.println("Touchpad");
    butt[PS_TOUCHPAD] = 0;
  }
  if (butt[PS_BUTTON] == 1)
  {
    Serial.println("PS Button");
    butt[PS_BUTTON] = 0;
  }
  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////

void drive(double A_x, double A_y, double ang_s)                                 //Drive Equation
{
  int F1, F2, F3;
  /*
     F3 is head and is connected to single channel MD
     131 is double channel and 129 is for single channel
  */
  F1 = (0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F2 = -(0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F3 = (0 * A_x) + (0.666 * A_y) - (0.333 * ang_s);

  F1 = map(F1, -200, 200, -32000, 32000);
  F2 = map(F2, -200, 200, -32000, 32000);
  F3 = map(F3, -200, 200, -32000, 32000);

  roboclaw.DutyM2(address1, -F1);
  roboclaw.DutyM2(address2, -F2);
  roboclaw.DutyM1(address2 , F3);

  Serial.print("\t");
  Serial.print(F1);
  Serial.print("\t");
  Serial.print(F2);
  Serial.print("\t");
  Serial.print(F3);
  Serial.print("\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

void powerwindowDuty(int duty)                                               //PowerWindow Throwing angle function
{
  roboclaw.DutyM1(address1, duty);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

int PID_drive(int currentValue_drive, int targetValue_drive)                //PID code block for DRIVE HARDCODED
{
  int error_drive = targetValue_drive - currentValue_drive;

  int prop_error_drive = error_drive;
  int integral_error_drive = error_drive + prev_error_drive;
  int diff_error_drive = error_drive - prev_error_drive;

  prev_error_drive = error_drive;
  return ((KP_drive * prop_error_drive) + (KI_drive * integral_error_drive) + (KD_drive * diff_error_drive));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

//int PID_driveIP(int currentValue_driveIP, int targetValue_driveIP)          //PID code block for DRIVE with IP
//{
//  int error_driveIP = targetValue_driveIP - currentValue_driveIP;
//
//  int prop_error_driveIP = error_driveIP;
//  int integral_error_driveIP = error_driveIP + prev_error_driveIP;
//  int diff_error_driveIP = error_driveIP - prev_error_driveIP;
//
//  prev_error_driveIP = error_driveIP;
//  return ((KP_driveIP * prop_error_driveIP) + (KI_driveIP * integral_error_driveIP) + (KD_driveIP * diff_error_driveIP));
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

int PID_throwing(int currentValue_throwing, int targetValue_throwing)      //PID code block for THROWING
{
  int error_throwing = targetValue_throwing - currentValue_throwing;

  int prop_error_throwing = error_throwing;
  int integral_error_throwing = error_throwing + prev_error_throwing;
  int diff_error_throwing = error_throwing - prev_error_throwing;

  prev_error_throwing = error_throwing;
  return ((KP_throwing * prop_error_throwing) + (KI_throwing * integral_error_throwing) + (KD_throwing * diff_error_throwing));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void gyroSetup()                                                                                       //Function for setting up both gyros
{
  digitalWrite(gyroSwitch1,HIGH);
  digitalWrite(gyroSwitch2,HIGH);

  mpu.setAddress(0x68); //drive
  mpu2.setAddress(0x69);  // throwing
  byte status = mpu.begin();
  byte status2 = mpu2.begin();
  Serial.print(F("MPU6050-1 status: "));
  Serial.println(status);
  Serial.print(F("MPU6050-2 status: "));
  Serial.println(status2);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(true,true); // gyro and accelero
  mpu2.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                                                         //Function for Throwing
int i = 0;
int j = 0;
int k = 250;
int l = 160;
void Throwing(int a, int b)
{
  digitalWrite(uForward,HIGH);
  digitalWrite(uBackward,LOW);

  digitalWrite(dBackward,HIGH);
  digitalWrite(dForward,LOW);

  analogWrite(uPin,a);
  analogWrite(dPin,b);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()                                                                                            // Setup for whole code
{
  ps_setup();

  roboclaw.begin(115200);
  Serial.begin(115200);
  Wire.begin();

  pinMode(centrePistonOut,OUTPUT);
  pinMode(centrePistonIn,OUTPUT);
  pinMode(leftPiston,OUTPUT);
  pinMode(rightPiston,OUTPUT);

  pinMode(uPin,OUTPUT);
  pinMode(dPin,OUTPUT);
  pinMode(uForward,OUTPUT);
  pinMode(uBackward,OUTPUT);
  pinMode(dForward,OUTPUT);
  pinMode(dBackward,OUTPUT);

  pinMode(gyroSwitch1,OUTPUT);
  pinMode(gyroSwitch2,OUTPUT);

  gyroSetup();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()                                                                                              // Code which has to be on loop
{
  ps_read();

  mpu.update();
  mpu2.update();
  
  /*____________________________________________________________________________________________________*/
  // Minor target angle adjustment for throwing
  if(yj1 >= 100)
  {
    targetGyroAngle_throwing++;
  }
  else if(yj1 <= -100)
  {
    targetGyroAngle_throwing--;
  }

    /*____________________________________________________________________________________________________*/
  // Minor target angle adjustment for drive
  if(xj2 >= 100)
  {
    targetGyroAngle_drive++;
  }
  else if(xj2 <= -100)
  {
    targetGyroAngle_drive--;
  }

  /*____________________________________________________________________________________________________*/
  //Flag conditions for BOHH targets
  if(I_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(A_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(K_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(C_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(H_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(E_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(J_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(B_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(G_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(D_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  if(F_BOHH_Flag == 1)
  {
    targetGyroAngle_throwing = 0;
    targetGyroAngle_drive = 0;
  }
  
  /*____________________________________________________________________________________________________*/
  //Centre Piston Actuation
  if(centrePiston_Flag == 1)
  {
    digitalWrite(centrePistonOut,HIGH);
    digitalWrite(centrePistonIn,LOW);
    if(millis()-time_centre == 1000)
    {
      digitalWrite(centrePistonOut,LOW);
      digitalWrite(centrePistonIn,HIGH);
      centrePiston_Flag = 0;
    }
  }
  else if(centrePiston_Flag == 0)
  {
    digitalWrite(centrePistonOut,LOW);
    digitalWrite(centrePistonIn,HIGH);
  }

  /*____________________________________________________________________________________________________*/
  //Left Piston Actuation
  if(leftPiston_Flag == 1)
  {
    digitalWrite(leftPiston,HIGH);
    if(millis()-time_left == 3000)
    {
      digitalWrite(leftPiston,LOW);
      leftPiston_Flag = 0;
    }
  }
  else if(leftPiston_Flag == 0)
  {
    digitalWrite(leftPiston,LOW);
  }

  /*____________________________________________________________________________________________________*/
  //Right Piston Actuation
  if(rightPiston_Flag == 1)
  {
    digitalWrite(rightPiston,HIGH);
    if(millis()-time_right == 3000)
    {
      digitalWrite(rightPiston,LOW);
      rightPiston_Flag = 0;
    }
  }
  else if(rightPiston_Flag == 0)
  {
    digitalWrite(rightPiston,LOW);
  }

  /*____________________________________________________________________________________________________*/
  //Throwing
  if(buttonCount_throwing%2 != 0)
  {
    digitalWrite(gyroSwitch1,LOW);
    digitalWrite(gyroSwitch2,LOW);
    
    Throwing(i,j);
    if(i<250 && i>=0)
    {
      i += 10;
    }
    else if(i>=250)
    {
      i = 250;
    }
    //////////////////
    if(j<160 && j>=0)
    {
      j += 10;
    }
    else if(i>=160)
    {
      j = 160;
    }
  }
  else if(buttonCount_throwing%2 == 0 && buttonCount_throwing != 0)
  {
    Throwing(k,l);
    if(k<=250 && k>0)
    {
      k -= 10;
    }
    else if(k <= 0)
    {
      k = 0;
    }
    //////////////////
    if(j<=160 && j>0)
    {
      j -= 10;
    }
    else if(j <= 0)
    {
      j = 0;
    }
  }
  
  /*____________________________________________________________________________________________________*/
  //Angle of gyros
  int driveGyroAngle = mpu.getAngleZ();
  int throwingGyroAngle = mpu2.getAngleZ();

  //Value of PIDs
  int throwingPIDval = PID_throwing(throwingGyroAngle, targetGyroAngle_throwing);
  int drivePIDval = PID_drive(driveGyroAngle, targetGyroAngle_drive);

  /*____________________________________________________________________________________________________*/
  
//  Serial.print(targetGyroAngle_throwing);
//  Serial.print("\t");
//  Serial.print(targetGyroAngle_drive);
//  Serial.print("\t");
//
//  Serial.print("Drive Angle: :");
//  Serial.print(driveGyroAngle );
// // Serial.print(" Throwing Angle: :");
// // Serial.print(throwingGyroAngle);
//  Serial.print("\t");
//  
//  Serial.print(throwingPIDval);
//  Serial.print("\t");
//  Serial.println(drivePIDval);
//  Serial.print(pot1);
//  Serial.print("\t");
//  Serial.println(pot2);
//  Serial.println(buttonCount_throwing);
  
  /*____________________________________________________________________________________________________*/

}
