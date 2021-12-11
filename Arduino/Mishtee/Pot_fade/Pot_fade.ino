int ledPin=3;
void setup()
{
  pinMode(ledPin,OUTPUT);
}
void loop()
{
  int analogValue = analogRead(A0);
  int brightness = map (analogValue,0,1023,0,255);
  analogWrite(ledPin,brightness);
}
