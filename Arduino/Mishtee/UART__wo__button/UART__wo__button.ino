//send
/*char mystr[5]="Hello";
void setup()
{
  Serial.begin (9600);
}
void loop()
{
  Serial.write(mystr,5);
  delay(1000);
}

 //recieve
char mystr[10];
void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  Serial.readBytes(mystr,5);
  Serial.println(mystr);
  delay(1000);
}
