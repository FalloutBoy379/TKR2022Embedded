int ledPin = 13;

void setup()
{
  //setting serial object
  Serial.begin(9600);
  //Assigning pin 13 as output
  pinMode(ledPin,OUTPUT);
}

void loop()
{
  //Waiting till Arduino recieves input
  while(Serial.available() == 0);

  //Read input
  int val = Serial.read() - '0';

  //loop for taking input and giving output by led on/off and statement print
  if(val == 1)
  
  {
    Serial.println("LED is ON");
    digitalWrite(ledPin,HIGH);
  }
  else if(val == 0)
  {
    Serial.println("LED is OFF");
    digitalWrite(ledPin,LOW);
  }
  else
  {
    Serial.println("invalid value");
  }

  Serial.flush();
}
