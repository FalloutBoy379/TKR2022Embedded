/*
   Hardware Timer as an Encoder interface.

   The STM32 Timers have the possibility of being used as an encoder interface.
   This can be both a quadrature encoder (mode 3) or pulse encoder with a signal to give direction (modes 1 and 2).
   The default behavior is for quadrature encoder.

   To avoid overflowing the encoder (which may or may not happen (although with 16 bits, it's likely), the following code
   will interrupt every time the number of pulses that each revolution gives to increment/decrement a variable (ints).

   This means that the total number of pulses given by the encoder will be (ints * PPR) + timer.getCount()

   Attached is also a bit of code to simulate a quadrature encoder.
   To test this library, make the connections as below:

   TIMER2 inputs -> Digital Pins used to simulate.
   D2 -> D4
   D3 -> D5

   COUNTING DIRECTION:
   0 means that it is upcounting, meaning that Channel A is leading Channel B

   EDGE COUNTING:

   mode 1 - only counts pulses on channel B
   mode 2 - only counts pulses on Channel A
   mode 3 - counts on both channels.

*/



#include "HardwareTimer.h"
//Encoder stuff

//Pulses per revolution
#define PPR 1024

HardwareTimer timer(1);

unsigned long ints = 0;

int potPin = PA6;

void func() {
  if (timer.getDirection()) {
    ints--;
  } else {
    ints++;
  }
}


void setup() {
  Serial2.begin(115200);
  Serial.begin(115200);
  //define the Timer channels as inputs.
  pinMode(PA9, INPUT);  //channel A
  pinMode(PA8, INPUT);  //channel B
  pinMode(potPin,INPUT);

  //configure timer as encoder
  timer.setMode(1, TIMER_ENCODER); //set mode, the channel is not used when in this mode (but it must be [1..4]).
  timer.pause(); //stop...
  timer.setPrescaleFactor(1); //normal for encoder to have the lowest or no prescaler.
  timer.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps.
  timer.setCount(0);          //reset the counter.
  timer.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction.
  timer.attachInterrupt(0, func); //channel must be 0 here
  timer.resume();                 //start the encoder...
}

//Support variables.
unsigned long interval = 0; //variable for status updates...
char received = 0;

void loop() {
  int potValue = analogRead(potPin);
//  long int current = micros();
  Serial2.write('_');
  delay(5);
  int count = timer.getCount();
  Serial2.write(highByte(count));
  delay(5);
  Serial2.write(lowByte(count));
  delay(5);
  Serial2.write('|');
  delay(5);
  Serial2.write(highByte(ints));
  delay(5);
  Serial2.write(lowByte(ints));
  delay(5);
  Serial2.write('#');
  delay(5);
  Serial2.write(highByte(potValue));
  delay(5);
  Serial2.write(lowByte(potValue));
  delay(5);
  int en1 = timer.getCount();
  int en1_f = (1024 * ints) + en1;
    Serial.println(en1_f/(3.6*4.1));
    Serial.println("\t");
    Serial.println(potValue);
//  Serial.println(micros() - current);
  //    interval = millis(); //update interval for user.
  //  }

}
