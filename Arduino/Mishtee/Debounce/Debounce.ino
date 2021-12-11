int switchPin=8;
int ledPin= 13;
boolean lastbutton = LOW;
boolean ledOn = false;
boolean currentbutton = LOW; //for debounce
void setup()
{
  pinMode(switchPin,INPUT);
  pinMode(ledPin, OUTPUT);
}
//code for debouncing starts here
boolean debounce(boolean last)
{
  boolean current = digitalRead(switchPin);
  if(last != current)
  {
    delay(5);
    current = digitalRead(switchPin);
  }
  return current;
}

void loop()
{
  currentbutton = debounce(lastbutton);
  if(lastbutton == LOW && currentbutton == HIGH)
  {
   ledOn = !ledOn ;
  }
  lastbutton = currentbutton;
  digitalWrite(ledPin,ledOn);
}
 
