int switchpin=8;
int ledpin=13;
  

void setup() {
  // put your setup code here, to run once:
  pinMode(switchpin,INPUT);
  pinMode(ledpin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(switchpin)==HIGH)
  {
    digitalWrite(ledpin,HIGH);
  }
  else
  {
    digitalWrite(ledpin,LOW);  
  }
}
