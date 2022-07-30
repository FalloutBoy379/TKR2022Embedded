#include <RoboClaw.h>

int potPin = A0;

RoboClaw roboclaw(&Serial1, 10000);
#define address 128
 //M1 and M2 are two channels on roboclaw ,that are to be input in syntax while defining PWM
void setup()
{
  roboclaw.begin(115200);
  Serial.begin(9600);

  pinMode(potPin,INPUT);
}

void loop()
{
  int val = analogRead(potPin);
  
  //Waiting till Arduino recieves input
  if(Serial.available()){
  //Read input
  int state = Serial.read() - '0';

  if (state == 0)
  {
    Serial.println("Motor is OFF");
    roboclaw.DutyM1(address,0);     // stop motor
  }

  else if (state == 1)
  {
    Serial.println("Motor Forward");
    roboclaw.DutyM1(address,24000);             // motor forward at 24000
  }

  else if (state == 2)
  {
    Serial.println("Motor Backward");
    roboclaw.DutyM1(address,-24000);            // motor backward at -24000   
  }

  else
  {
    Serial.println("Invalid command");
  }

  Serial.println(val);
  // Serial.flush();
  }
}


// Also has roboclaw.ForwardM1(any value from 0 to 127)
// or roboclaw.BackwardM1(any value from 0 to 127)
// or roboclaw.ForwardBackwardM1(value from 64 to 127 is forward while 64 to 0 is backward)
// Duty has range of -32000 to 32000 with 0 being stop
