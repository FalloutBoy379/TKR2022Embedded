#define enA 9
#define in1 6
#define  in2 7
#define button 4

boolean buttonState=LOW;
int rotDirection=0;
int pressed= false;

void setup()
{
  pinMode(enA,OUTPUT);
  pinMode(in1,INPUT);
    pinMode(in2,INPUT);
    pinMode(button,INPUT);
  digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
}

void loop()
{
  Serial.begin(9600);
  int potValue=analogRead(A0);
    int pwmOutput=map(potValue,0,1023,0,255);
    analogWrite(enA,pwmOutput);
    Serial.print("Analog: ");
  Serial.print(potValue);
  
    if(digitalRead(button)==true)
    {
      pressed=!pressed;
    }
  while(digitalRead(button)==true);
  delay(20);
  if(pressed==true & rotDirection==0)
  {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    rotDirection=1;
    delay(20);
  }
  if(pressed==false & rotDirection==1)
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    rotDirection=0;
    delay(20);
  }
  
  






}
