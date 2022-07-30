int ledPin = 13;
int switchButton = 8;
boolean lastButton = LOW;
boolean ledOn = false;

void setup()
{
 pinMode(ledPin,OUTPUT);
 pinMode(switchButton,INPUT);
}

void loop()
{
  if(digitalRead(switchButton) == HIGH && lastButton == LOW)
  {
    ledOn = !ledOn;
  }
  else
  {
   lastButton = digitalRead(switchButton);
  }
  digitalWrite(ledPin,ledOn);
}
