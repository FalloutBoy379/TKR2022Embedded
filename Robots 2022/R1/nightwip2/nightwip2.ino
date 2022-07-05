//#encoder with 2 wheels on

#define KPE 0.15
#define KIE 0.1
#define KDE 7

#define REDLED 4
#define GREENLED 5
#define BLUELED 6

#define ENA 2
#define ENB 11

int sudhaar = 0;
volatile int count = 0;
#include <RoboClaw.h>

void lights_setup();

void lights_write(int r, int g, int b);

unsigned long int disconnectedcounter = 0;
long long a = 0;
RoboClaw roboclaw(&Serial1, 10000);

#define address1 128
#define address2 129

///////////////////////////////////////////////////////////////////////////////////////////
//Defined pins for Throwing

int r = 0;
int g = 255;
int b = 0;

int enccounttracker = 0;


String RPIVAL = "";
int RPICORRECTION  = 0 ;


int uPin = 8;
int uForward = 42;
int uBackward = 43;

int dPin = 9;
int dForward = 41;
int dBackward = 40;

volatile int buttonCount_throwing = 0;
int targetcount = 0;//button count for odd||even counts for starting or stopping Throwing
int duty = 0;
///////////////////////////////////////////////////////////////////////////////////////////
//Defined pins for pistons
int centrePistonOut = 28;
int centrePistonIn = 32;
int leftPiston = 26;
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
//Target variables for PID
int targetGyroAngle_throwing = 0;

///////////////////////////////////////////////////////////////////////////////////////////
//Flags for 11 BOHHs
int pelpendiculal_flag = 0;
int I_BOHH_Flag = 0;
int A_BOHH_Flag = 0;
int K_BOHH_Flag = 0;
int C_BOHH_Flag = 0;

int Up_flag = 0;
int Down_flag = 0;
long long Up = 0;
long long Down = 0;

/////////////////////////////////////////////////////
int PID_drive(int currentValue_drive, int targetValue_drive);

//////////////////////////////////////////////////////////////////////////////////////////

#define KP_throwing 0                                         //PID Values for THROWING
#define KI_throwing 0
#define KD_throwing 0
int prev_error_throwing = 0;

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

int IPFLAG = 0;

int encresetflag = 1;
//int drive

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
////////////////////////////////////////////////////////////////////////////////////////////////////
void encoder() {
  if (digitalRead(ENB))
  {
    count++;
  }
  else
  {
    count--;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
void powerwindowDuty()                                                      //PowerWindow Throwing angle function
{
  roboclaw.DutyM1(address1, duty);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to read data from PS : ps_read()
//READ DATA FROM CONTROLLER
bool ps_read() {
  RX_count = 0;
  disconnectedcounter++;
  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, pwm_range, (-pwm_range));
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));
  pot1 = map(RX[4], 0, RX_range, 0, (pwm_range));          //R2
  pot2 = map(RX[5], 0, RX_range, 0, (pwm_range));          //L2

  if (butt[PS_SHARE] == 1)
  {
    Serial.println("Share");
    butt[PS_SHARE] = 0;
  }
  if (butt[PS_OPTIONS] == 1)
  {
    Serial.println("Options");
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
    IPFLAG = 1;
    //pelpendiculal_flag = 1;
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
  int F1, F2, F3;

  F1 = (0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F2 = -(0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  // F3 = (0*A_x) - (0.666*A_y) + (0.333*ang_s);

  F1 = F1 * 150;
  F2 = F2 * 150;


  if (F1 > 15000)
  {
    F1 = 15000;
  }
  if (F2 > 15000)
  {
    F2 = 15000;
  }
  if (F1 < -15000)
  {
    F1 = -15000;
  }
  if (F2 < -15000)
  {
    F2 = -15000;
  }

  roboclaw.DutyM2(128, F1);
  roboclaw.DutyM2(129, -F2);


  Serial.print(F1);
  Serial.print("\t");
  Serial.print(F2);
  Serial.print("\t");
  Serial.print(count);
  Serial.print("\n");


}
/////////////////////////////////////////////////////////////////////////////////////////////

int error_drive = 0;
int integral_error_drive = 0;
int prev_error_drive = 0;

int PID_drive(int currentValue_drive, int targetValue_drive)                //PID code block for DRIVE HARDCODED
{
  error_drive = targetValue_drive - currentValue_drive;

  int prop_error_drive = error_drive;
  integral_error_drive = error_drive + prev_error_drive;
  int diff_error_drive = error_drive - prev_error_drive;

  prev_error_drive = error_drive;
  return ((KPE * prop_error_drive) + (KIE * integral_error_drive) + (KDE * diff_error_drive));
}
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function for Throwing
int i = 0;
int j = 0;
int k = 240;
int l = 200;
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


  pinMode(ENA, INPUT_PULLUP);
  pinMode(ENB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENA), encoder, RISING);

  roboclaw.begin(115200);

  Serial.begin(115200);
  pinMode(centrePistonOut, OUTPUT);
  pinMode(centrePistonIn, OUTPUT);
  pinMode(leftPiston, OUTPUT);
  pinMode(rightPiston, OUTPUT);

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
}

void loop()                                                                                              // Code which has to be on loop
{
  if (RX_count == 1)
  {
    ps_read();

    if(Serial.available())
    {
    RPIVAL = Serial.readStringUntil('\n');
    RPICORRECTION  = RPIVAL.toInt();

    Serial.print("x difference value: ");
    Serial.println(RPICORRECTION);
    Serial.print("wheel values: ");
    Serial.println(RPICORRECTION * 50);
    }

    r = r + 30;
    g = g - 20;
    b = b + 69;

    if (r >= 255)
    {
      r = 0;
    }

    if (b >= 255)
    {
      b = 0;
    }

    if (g <= 0)
    {
      g = 255;
    }
    lights_write(r, g, b);

    if (Up_flag == 1)
    {
      encresetflag = 1;
      duty = -20000;
      powerwindowDuty();
      if (millis() - Up > 100)
      {

        duty = 0;
        powerwindowDuty();
        Up_flag = 0;
      }
    }
    if (Down_flag == 1)
    {
      encresetflag = 1;
      duty = 20000;
      powerwindowDuty();
      if (millis() - Down > 100)
      {
        duty = 0;
        powerwindowDuty();
        Down_flag = 0;
      }

    }

    /*____________________________________________________________________________________________________*/
    //Flag conditions for BOHH targets

    if (I_BOHH_Flag == 1)
    {
      encresetflag = 1;
      targetGyroAngle_throwing = 0;
      // targetGyroAngle_drive = 29;
      I_BOHH_Flag = 0;
    }
    if (A_BOHH_Flag == 1)
    {
      encresetflag = 0;
      targetcount = 640;// C flag
      targetGyroAngle_throwing = 0;
      // targetGyroAngle_drive = -30;
      A_BOHH_Flag = 0;
    }

    if (K_BOHH_Flag == 1)//R1
    {
      encresetflag = 0;
      targetcount = 0;
      enccounttracker = targetcount;
      targetGyroAngle_throwing = 0;
      //targetGyroAngle_drive = 62;
      K_BOHH_Flag = 0;
    }
    if (C_BOHH_Flag == 1)//L1
    {
      encresetflag = 0;
      targetcount = -910;//90 degree
      enccounttracker = targetcount;
      targetGyroAngle_throwing = 0;
      // targetGyroAngle_drive = -65;
      C_BOHH_Flag = 0;
    }


    /*____________________________________________________________________________________________________*/
    //Centre Piston Actuation
    if (centrePiston_Flag == 1)
    {
      encresetflag = 1;
      digitalWrite(centrePistonOut, HIGH);
      digitalWrite(centrePistonIn, LOW);
      if (millis() - time_centre > 69

         )
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
      encresetflag = 1;
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

      encresetflag = 1;
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
    //    if (pelpendiculal_flag == 1)
    //    {
    //      //targetcount = 1000;
    //      //targetGyroAngle_drive = 90;
    //      pelpendiculal_flag = 0;
    //    }
    /*____________________________________________________________________________________________________*/
    //Throwing
    if (buttonCount_throwing % 2 != 0)
    {
      encresetflag = 1;
      k = 255;
      l = 255;

      Throwing(k, l);

    }
    else if (buttonCount_throwing % 2 == 0 && buttonCount_throwing != 0)
    {
      k = 0;
      l = 0;
      Throwing(k, l);

      i = 0;
      j = 0;
      encresetflag = 0;
      count = enccounttracker;
    }

    sudhaar = PID_drive(count, targetcount);

    if (encresetflag == 0)
    {
      drive(0, 0, sudhaar  + RPICORRECTION);
    }
    else
    {

    }

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
