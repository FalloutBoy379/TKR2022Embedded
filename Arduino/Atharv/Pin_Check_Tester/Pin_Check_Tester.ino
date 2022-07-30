int a = 43;

void setup()
{
  pinMode(a,OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  while(Serial.available() == 0);

  char mode = Serial.read();
  int i;
  if(mode =='1')
  {
    Serial.println("on");
    digitalWrite(a,HIGH);
  }

  else if(mode =='0')
  {
    Serial.println("off");
    digitalWrite(a,LOW);
  }

  else
  {
    Serial.println("invalid");
  }
}
