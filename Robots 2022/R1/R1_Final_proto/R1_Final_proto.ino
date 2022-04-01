#include <RoboClaw.h>
#include <FastLED.h>

FASTLED_USING_NAMESPACE
#define DATA_PIN    8
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    29
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120

int potPin = A0;

RoboClaw roboclaw(&Serial1, 10000);
#define address1 128
#define address2 129

#define EN1A 2
#define EN1B 10
#define EN2A 3
#define EN2B 11
volatile int count1 = 0;
volatile int count2 = 0;

#define KP 0.9
#define KI 0.0
#define KD 1
int prev_error = 0;

int PID(int currentValue, int targetValue);

int duty = 0;
int i;

int upPin = 5;
int dpPin = 6;
int dForward = 43;
int dBackward = 44;
int uForward = 41;
int uBackward = 40;

#define pistonCenter 30
#define pistonLeft 31
#define pistonRight 32
#define motorLeftA 13
#define motorLeftB 9
#define motorRightA 22
#define motorRightB 24
#define pwm 12


uint8_t button_count = 0;
uint8_t counter = 0;


//Call ps_setup() in void setup()
//Call ps_read() in void loop()



#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))
#define pwm_range 30000
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


void encoder1()
{
  if (digitalRead(EN1B) == HIGH)
  {
    count1++;
  }
  else if (digitalRead(EN1B) == LOW)
  {
    count1--;
  }
}
void encoder2()
{
  if (digitalRead(EN2B) == HIGH)
  {
    count2++;
  }
  else if (digitalRead(EN2B) == LOW)
  {
    count2--;
  }
}

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
    duty = 20000;
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
    duty = -20000;
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
    duty = 0;
    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    Serial.println("right");
    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1)
  {
    button_count++ ;
    Serial.println("square");
    butt[PS_SQUARE] = 0;
  }
  if (butt[PS_CIRCLE] == 1)
  {
    setLED(0, 255, 0);
    count1 = 0;
    button_count = 0;
    counter = 0;
    butt[PS_CIRCLE] = 0;
  }
  if (butt[PS_TRIANGLE] == 1)
  {
    setLED(0, 255, 0);
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
    setLED(255, 0, 0);
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
    Serial.println("L3");
    butt[PS_L3] = 0;
  }
  if (butt[PS_R3] == 1)
  {
    Serial.println("R3");
    butt[PS_R3] = 0;
  }
}

void drive(double A_x, double A_y, double ang_s)
{
  int F1, F2, F3;
  /*
     F3 is head and is connected to single channel MD
     131 is double channel and 129 is for single channel
  */
  F1 = (0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F2 = -(0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F3 = (0 * A_x) - (0.666 * A_y) + (0.333 * ang_s);

  F1 = map(F1, -127, 127, -32000, 32000);
  F2 = map(F2, -127, 127, -32000, 32000);
  F3 = map(F3, -127, 127, -32000, 32000);

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

void powerwindowDuty()
{
  roboclaw.DutyM1(address1, duty);
}

int PID(int currentValue, int targetValue)
{
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error;
  int diff_error = error - prev_error;

  prev_error = error;
  return ((KP * prop_error) + (KI * integral_error) + (KD * diff_error));
}

void delay_ms(long int ms) {
  long int currentMillis = millis();
  while (millis() - currentMillis < ms) {
    ps_read();
  }
}

void firstActuation() {
  Serial.println("First");
  digitalWrite(pistonCenter, HIGH);
  delay_ms(500);
  digitalWrite(pistonCenter, LOW);
  delay_ms(100);
  digitalWrite(pistonLeft, HIGH);
  motorLeft(1, 255);
  delay_ms(3000);
  digitalWrite(pistonLeft, LOW);
  motorLeft(0, 0);
}

void secondActuation() {
  Serial.println("Second");
  digitalWrite(pistonCenter, HIGH);
  delay_ms(500);
  digitalWrite(pistonCenter, LOW);
  delay_ms(100);
  digitalWrite(pistonRight, HIGH);
  motorRight(1, 255);
  delay_ms(3000);
  digitalWrite(pistonRight, LOW);
  motorRight(0, 0);
}

void thirdActuation() {
  Serial.println("Third");
  digitalWrite(pistonCenter, HIGH);
  delay_ms(500);
  digitalWrite(pistonCenter, LOW);
}



void motorRight(bool state, uint8_t vel) {
  digitalWrite(motorRightA, state);
  digitalWrite(motorRightB, LOW);
  analogWrite(pwm, vel);
}

void motorLeft(bool state, uint8_t vel) {
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
  
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(r,g,b);
    FastLED.show();
    delay(msPerLed);
    Serial.println(msPerLed);
    
  }
}

void loadingLEDrev(int r, int g, int b, int ms) {
  long int current = millis();
  int msPerLed = ms / NUM_LEDS;
  int i = 0;
  
  for(int i=NUM_LEDS;i>0;i--){
    leds[i] = CRGB(r,g,b);
    FastLED.show();
    delay(msPerLed);
    Serial.println(msPerLed);
    
  }
}

void setup()
{
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  setLED(255, 0, 0);

  ps_setup();
  roboclaw.begin(115200);
  Serial.begin(115200);

  pinMode(pistonCenter, OUTPUT);
  pinMode(pistonRight, OUTPUT);
  pinMode(pistonLeft, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);
  

  pinMode(upPin, OUTPUT);
  pinMode(dpPin, OUTPUT);
  pinMode(dForward, OUTPUT);
  pinMode(dBackward, OUTPUT);
  pinMode(uForward, OUTPUT);
  pinMode(uBackward, OUTPUT);
  pinMode(potPin, INPUT);

  pinMode(EN1A, INPUT_PULLUP);
  pinMode(EN1B, INPUT_PULLUP);
  pinMode(EN2A, INPUT_PULLUP);
  pinMode(EN2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EN1A), encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(EN2A), encoder2, RISING);
  delay(1000);
  loadingLED(0, 255, 0, 1500);

}

int curval = 0;

void loop()
{
  duty = 0;
  ps_read();
  int motor_val = analogRead(potPin);

  powerwindowDuty();

  curval = PID((count1), 0);
  drive(0, 0, curval);

  if (button_count == 1 && counter == 0) {
    setLED(255, 0, 255);
    firstActuation();
    counter = 1;
  }
  else if (button_count == 2 && counter == 1) {
    setLED(0, 255, 255);
    secondActuation();
    counter = 2;

  }
  else if (button_count == 3 && counter == 2) {
    setLED(255, 255, 0);
    counter = 3;
    thirdActuation();
  }

  //  Serial.print("PID pot value\t");
  //  Serial.print(motor_val);
  Serial.print("PW duty \t");
  Serial.print(duty);
  Serial.print("\t enc count \t");
  Serial.print((count1));


}

void loop1()
{
  loadingLED(255, 0, 0, 500);
  delay(100);
  loadingLEDrev(255, 255, 0,500);
  delay(100);
//  ps_read();
//  Serial.print(xj1);
//  Serial.print("\t");
//  Serial.print(xj2);
//  Serial.print("\t");
//  Serial.print(yj1);
//  Serial.print("\t");
//  Serial.println(yj2);
}
