// C++ code

int m1 = 2;
int m2 = 3;
int m3 = 4;
int m4 = 5;
void setup()
{
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
}

void loop()
{
  //forward
  digitalWrite(m1,HIGH);
  digitalWrite(m2,LOW);
  digitalWrite(m3,HIGH);
  digitalWrite(m4,LOW);
  delay(10000);
  
  //backward
  digitalWrite(m2,HIGH);
  digitalWrite(m1,LOW);
  digitalWrite(m4,HIGH);
  digitalWrite(m3,LOW);
  delay(10000);
  
  
  //Right
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, LOW);
  delay(10000);
  
  //left
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, LOW);
  delay(10000);


    //stop
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, LOW);
  delay(10000);
  
}
