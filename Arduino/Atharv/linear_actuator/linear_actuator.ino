int pwmPin = 9;
int mForward = 7;
int mBackward = 8;
int pot = A1;
int potValue = 0;


void forward_motion(void)
{
  analogWrite(pwmPin,500);
  digitalWrite(mForward,HIGH);
  digitalWrite(mBackward,LOW);
}

void backward_motion(void)
{
  analogWrite(pwmPin,500);
  digitalWrite(mForward,LOW);
  digitalWrite(mBackward,HIGH);
}

void brake(void){
  digitalWrite(mForward, HIGH);
  digitalWrite(mBackward, HIGH);
}
void setup()
{
  Serial.begin(9600);
  pinMode(pwmPin,OUTPUT);
  pinMode(mForward,OUTPUT);
  pinMode(mBackward,OUTPUT);
  pinMode(pot,INPUT);
}

void loop()
{
  forward_motion();
  delay(5000);
  brake();
  delay(500);
  backward_motion();
  delay(5000);
  brake();
  delay(500);
  
  potValue = analogRead(pot);
  Serial.println(potValue);
}
