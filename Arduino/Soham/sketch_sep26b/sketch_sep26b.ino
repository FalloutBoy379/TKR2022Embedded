uint8_t Motor1pin1=D0;
uint8_t Motor1pin2=D1;
uint8_t Motor2pin1=D5;
uint8_t Motor2pin2=D6;
void setup() {
  // put your setup code here, to run once:
  pinMode(Motor1pin1,OUTPUT);
  pinMode(Motor1pin2,OUTPUT);
  pinMode(Motor2pin1,OUTPUT);
  pinMode(Motor2pin2,OUTPUT);

}

void loop() 
{
  // put your main code here, to run repeatedly:
  
  digitalWrite(Motor1pin1,LOW);
  digitalWrite(Motor1pin2,HIGH);
  digitalWrite(Motor2pin1,LOW);
  digitalWrite(Motor2pin2,HIGH);
  delay(5000);
  digitalWrite(Motor1pin1,HIGH);
  digitalWrite(Motor1pin2,LOW);
  digitalWrite(Motor2pin1,HIGH);
  digitalWrite(Motor2pin2,LOW);
  delay(5000);
}
