int pwmpin = 9;
int dirf = 7;
int dirb = 8;
int enca = 2;
int encb = 3;
int lenca;
int cenca;//current enc A
int cencb;//current enc B
volatile int enccount=0;
void setup()
{
  Serial.begin(9600);
  pinMode(pwmpin, OUTPUT);
  pinMode(dirf, OUTPUT);
  pinMode(dirb, OUTPUT);
  pinMode(enca, INPUT);
  pinMode(encb, INPUT);
}

void loop() {
  analogWrite(pwmpin, 255);
  digitalWrite(dirb, HIGH);
  digitalWrite(dirf, LOW);
  attachInterrupt(digitalPinToInterrupt(enca),encoder,RISING);
  Serial.print(enccount);
  Serial.print("\n");
}
void encoder()
{
  cencb = digitalRead(encb);
  if (cencb !=1)
  {
    enccount ++;
  }
  else if (cencb ==1)
  {
    enccount --;
  } 
}
