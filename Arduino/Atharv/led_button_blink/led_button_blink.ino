int switchPin = 8;
int ledPin = 13;

void setup() {
  pinMode(switchPin,INPUT);
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  Serial.println(digitalRead(switchPin));
  delay(1000);
  if (digitalRead(switchPin) == HIGH)
  {
    digitalWrite(ledPin,HIGH);
  }
  else
  {
    digitalWrite(ledPin,LOW);
  }
}
