#include <RoboClaw.h>

#define potPin A0    // select the input pin for the potentiometer

RoboClaw roboclaw(&Serial1,10000);
#define address 128

#define KP 350 
#define KI 0
#define KD 0
int prev_error=0;

// int potVal[100];  // variable to store the value coming from the sensor
int potVal;

void setup() {
  // declare the potPin as an OUTPUT:
  roboclaw.begin(115200);
  Serial.begin(115200);
  pinMode(potPin, INPUT);
  analogReference(INTERNAL2V56);         //Sets arduino reference voltage to 2.56V for ADC
}

int i=0;
int target = 340;

void loop() {
  // read the value from pot:
  
//  potVal[i] = analogRead(potPin);
//  i++;
//  if(i>99){
//    i=0;
//  }
  potVal = analogRead(potPin);

  long int sum=0;
  for(int j=0; j<100;j++){
    sum += analogRead(potPin);
  }
  int avgVal = (sum/99);
  
  int PIDValue = PID(avgVal, target);
  
  if (PIDValue < -30000)
  {
    PIDValue = -30000;  
  }
  else if(PIDValue > 30000)
  {
    PIDValue = 30000;
  }
  else if(PIDValue < 20000)
  {
    PIDValue = 0;
  }
  else if(PIDValue > -20000)
  {
    PIDValue = 0;
  }
  
  roboclaw.DutyM1(address,PIDValue);

  Serial.print(potVal);
  Serial.print("\t");
  Serial.println(PIDValue);
}

int PID(int currentValue, int targetValue)
{
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error;
  int diff_error = error-prev_error;

  prev_error = error;
  return ((KP*prop_error)+(KI*integral_error)+(KD*diff_error));
}
