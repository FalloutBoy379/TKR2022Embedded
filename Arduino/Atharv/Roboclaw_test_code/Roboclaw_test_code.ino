#include "RoboClaw.h"-

RoboClaw roboclaw(&Serial1,10000);

#define address1 128

 
void setup()
{
  roboclaw.begin(115200);
  Serial.begin(9600);

}

void loop()
{
  roboclaw.DutyM1(address1,16000);
  delay(2000);
  roboclaw.DutyM1(address1,16000);
  delay(2000);
}
