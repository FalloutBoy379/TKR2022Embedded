#include <RoboClaw.h>
#include <FastLED.h>
#include "Wire.h"
#include <MPU6050_light.h>

FASTLED_USING_NAMESPACE           //for LED strip
#define DATA_PIN    8
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    29
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120

MPU6050 mpu(Wire);              //Initialize Gyro
long timer = 0;

int gyroAngleZ = 0;

int target_gyro = 0;           // Drive PID
int curval = 0;

RoboClaw roboclaw(&Serial1, 10000);
#define address1 128
#define address2 129

//#define EN1A 2                     //1A 1B is for Drive PID
//#define EN1B 10
//#define EN2A 3                     //2A 2B is for PowerWindow PID
//#define EN2B 11
volatile int count1 = 0;
volatile int count2 = 0;

#define KP 15
#define KI 5
#define KD 3
int prev_error = 0;

int PID(int currentValue, int targetValue);

int duty = 0;
int i;

int upPin = 5;                                 //Throwing Mechanism
int dpPin = 6;
int dForward = 43;
int dBackward = 42;
int uForward = 41;
int uBackward = 40;

#define pistonCenterOut 50                        //Hopper Mechanism
#define pistonCenterIn 51   
#define pistonLeft 52
#define pistonRight 53
#define motorLeftA 13
#define motorLeftB 9
#define motorRightA 22
#define motorRightB 24
#define pwm 12


uint8_t button_count = 0;                       //Hopper Button Count
uint8_t counter = 0;


//Standard PS Code

//Call ps_setup() in void setup()
//Call ps_read() in void loop()

#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))
#define pwm_range 200
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


//void encoder1()
//{
//  if (digitalRead(EN1B) == HIGH)
//  {
//    count1++;
//  }
//  else if (digitalRead(EN1B) == LOW)
//  {
//    count1--;
//  }
//}
//void encoder2()
//{
//  if (digitalRead(EN2B) == HIGH)
//  {
//    count2++;
//  }
//  else if (digitalRead(EN2B) == LOW)
//  {
//    count2--;
//  }
//}

//READ DATA FROM CONTROLLER
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
    duty = 23000;                                  //Throwing Mechanism PowerWindow Angle- Forward
    //    duty += 1000;
    //    if(duty <= -30000)
    //    {
    //      duty = -30000;
    //    }
    //    if (duty >= 30000)
    //    {
    //      duty = 30000;
    //    }
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    duty = -20000;                                 //Throwing Mechanism PowerWindow Angle- Backward
    //    duty -= 1000;
    //    if(duty < -30000)
    //    {
    //      duty = -30000;
    //    }
    //    if (duty > 30000)
    //    {
    //      duty = 30000;
    //    }
    butt[PS_DOWN] = 0;
  }
  if (butt[PS_LEFT] == 1)
  {
    firstActuation();
   butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    secondActuation();
    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1)
  {
    thirdActuation();
//    button_count++ ;                                //Hopper Mechanism Button
    butt[PS_SQUARE] = 0;
  }
  if (butt[PS_CIRCLE] == 1)
  {
    setLED(0, 255, 0);                              //Button for Drive Encoder value to be set as 0
    count1 = 0;                                     //Also Hopper button count to be set as 0
    count2 = 0;
    target_gyro = 0;
    button_count = 0;
    counter = 0;
    butt[PS_CIRCLE] = 0;
  }
  if (butt[PS_TRIANGLE] == 1)
  {
    setLED(0, 255, 0);                               //Throwing Mechanism motors OFF
    digitalWrite(uForward, HIGH);
    digitalWrite(uBackward, LOW);
    i = 255;
    while ( i > 0 )
    {
      analogWrite(upPin , i);
      i = i - 10;
    }
    digitalWrite(upPin, LOW);
    digitalWrite(dForward, LOW);
    digitalWrite(dBackward, HIGH);
    i = 165;
    while ( i > 0 )
    {
      analogWrite(dpPin , i);
      i = i - 10;
    }
    digitalWrite(dpPin, LOW);
  }
  butt[PS_TRIANGLE] = 0;

  if (butt[PS_CROSS] == 1)
  {
    setLED(255, 0, 0);                               //Throwing Mechanism motors ON
    digitalWrite(uForward, HIGH);
    digitalWrite(uBackward, LOW);
    i = 0;
    while ( i < 255)
    {
      analogWrite(upPin , i);
      i = i + 10;
    }


    digitalWrite(dForward, LOW);
    digitalWrite(dBackward, HIGH);
    i = 0;
    while ( i < 165)
    {
      analogWrite(dpPin , i);
      i = i + 10;
    }
  }
  butt[PS_CROSS] = 0;

  if (butt[PS_L1] == 1)
  {
    target_gyro += 10;                                  //Bot rotation by increasing target value
    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {
    target_gyro -= 10;                                  //Bot rotation by decreasing target value
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

void drive(double A_x, double A_y, double ang_s)         //Drive Equation
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

void powerwindowDuty()                                    //PowerWindow Throwing angle function
{
  roboclaw.DutyM1(address1, duty);
}

int PID(int currentValue, int targetValue)                //PID code block
{
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error;
  int diff_error = error - prev_error;

  prev_error = error;
  return ((KP * prop_error) + (KI * integral_error) + (KD * diff_error));
}

void delay_ms(long int ms) {                             //Delay byr millis
  long int currentMillis = millis();
  while (millis() - currentMillis < ms) {
    ps_read();
  }
}

void firstActuation() {                                  //Hopper Actuation
  Serial.println("First");                               //First Middle Piston is actuated
  digitalWrite(pistonCenterOut, HIGH);                   //Then left Motor for pushing the ball is turned ON
  digitalWrite(pistonCenterIn, LOW);
  curval = -PID((gyroAngleZ), target_gyro);                //Then left piston is actuated
  drive(0, 0, curval);                                   //Left motor is turned OFF
  ps_read();
  delay_ms(500);
  ps_read();
  digitalWrite(pistonCenterOut, LOW);
  digitalWrite(pistonCenterIn, HIGH);
  curval = -PID((gyroAngleZ), target_gyro);
  drive(0, 0, curval);
  ps_read();
  delay_ms(2000);
  ps_read();
  digitalWrite(pistonLeft, HIGH);
  curval = -PID((gyroAngleZ), target_gyro);
  drive(0, 0, curval);
  motorLeft(1, 255);
  ps_read();
  delay_ms(3000);
  ps_read();
  curval = -PID((gyroAngleZ), target_gyro);
  drive(0, 0, curval);
  digitalWrite(pistonLeft, LOW);
  motorLeft(0, 0);
}

void secondActuation() {                                  //Hopper Actuation
  Serial.println("Second");                               //Middle Piston is actuated
  digitalWrite(pistonCenterIn, LOW);
  digitalWrite(pistonCenterOut, HIGH);                     //Then right motor for pushing the ball is turned ON
  curval = -PID((gyroAngleZ), target_gyro);                 //Then right piston is actuated
  drive(0, 0, curval);                                    //Right motor is turned OFF
  ps_read();
  delay_ms(500);
  ps_read();
  curval = -PID((gyroAngleZ), target_gyro);
  drive(0, 0, curval);
  digitalWrite(pistonCenterOut, LOW);
  digitalWrite(pistonCenterIn, HIGH);
  ps_read();
  delay_ms(2000);
  ps_read();
  curval = -PID((gyroAngleZ), target_gyro);
  drive(0, 0, curval);
  digitalWrite(pistonRight, HIGH);
  motorRight(1, 255);
  ps_read();
  delay_ms(3000);
  ps_read();
  curval = -PID((gyroAngleZ), target_gyro);
  drive(0, 0, curval);
  digitalWrite(pistonRight, LOW);
  motorRight(0, 0);
}

void thirdActuation() {                                    //Middle Piston is Actuated
  Serial.println("Third");
  digitalWrite(pistonCenterOut, HIGH);
  digitalWrite(pistonCenterIn, LOW);
  ps_read();
  delay_ms(500);
  ps_read();
  curval = -PID((gyroAngleZ), target_gyro);
  drive(0, 0, curval);
  digitalWrite(pistonCenterOut, LOW);
  digitalWrite(pistonCenterIn, HIGH);
}



void motorRight(bool state, uint8_t vel) {                  //User defined function for Right hopper motor for pushing ball
  digitalWrite(motorRightA, state);
  digitalWrite(motorRightB, LOW);
  analogWrite(pwm, vel);
}

void motorLeft(bool state, uint8_t vel) {                   //User defined function for Left hopper motor for pushing ball
  digitalWrite(motorLeftA, state);
  digitalWrite(motorLeftB, LOW);
  analogWrite(pwm, vel);
}

void setLED(int r, int g, int b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    //    leds.fadeToBlackBy(40);
    leds[i] = CRGB(r, g, b);
  }
  FastLED.show();
}

void loadingLED(int r, int g, int b, int ms) {
  long int current = millis();
  int msPerLed = ms / NUM_LEDS;
  int i = 0;

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r, g, b);
    FastLED.show();
    delay(msPerLed);
    Serial.println(msPerLed);

  }
}

void loadingLEDrev(int r, int g, int b, int ms) {
  long int current = millis();
  int msPerLed = ms / NUM_LEDS;
  int i = 0;

  for (int i = NUM_LEDS; i > 0; i--) {
    leds[i] = CRGB(r, g, b);
    FastLED.show();
    delay(msPerLed);
    Serial.println(msPerLed);

  }
}

void setup()
{
  ps_setup();
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  setLED(255, 0, 0);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");

  roboclaw.begin(115200);
  Serial.begin(115200);

  pinMode(pistonCenterOut, OUTPUT);
  pinMode(pistonCenterIn, OUTPUT);
  pinMode(pistonRight, OUTPUT);
  pinMode(pistonLeft, OUTPUT);

  pinMode(upPin, OUTPUT);
  pinMode(dpPin, OUTPUT);
  pinMode(dForward, OUTPUT);
  pinMode(dBackward, OUTPUT);
  pinMode(uForward, OUTPUT);
  pinMode(uBackward, OUTPUT);

//  pinMode(EN1A, INPUT_PULLUP);
//  pinMode(EN1B, INPUT_PULLUP);
//  pinMode(EN2A, INPUT_PULLUP);
//  pinMode(EN2B, INPUT_PULLUP);

//  attachInterrupt(digitalPinToInterrupt(EN1A), encoder1, RISING);
//  attachInterrupt(digitalPinToInterrupt(EN2A), encoder2, RISING);
  delay_ms(1000);
  setLED(0, 255, 0);

}

void loop()
{

//  if (RX_count == 1){
 
  mpu.update();
  
  duty = 0;
  ps_read();

  powerwindowDuty();
  
  gyroAngleZ = mpu.getAngleZ();
  curval = -PID((gyroAngleZ), target_gyro);
//  drive(xj1, -yj1, yj2);
  drive(0, 0, curval);

//  if (button_count == 1 && counter == 0) {
//    setLED(255, 0, 255);
//    firstActuation();
//    counter = 1;
//  }
//  else if (button_count == 2 && counter == 1) {
//    setLED(0, 255, 255);
//    secondActuation();
//    counter = 2;
//
//  }
//  else if (button_count == 3 && counter == 2) {
//    setLED(255, 255, 0);
//    counter = 3;
//    thirdActuation();
//  }

  Serial.print(" Gyro_Z \t");
  Serial.print(mpu.getAngleZ());
  Serial.print("\t PID \t");
  Serial.print(PID((gyroAngleZ), target_gyro));
//  }  

}
