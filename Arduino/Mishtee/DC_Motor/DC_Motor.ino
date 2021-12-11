int i;
//motor a connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// motor b connections
int enB = 3;
int in3 = 5;
int in4 = 4;
void setup(){
  // set all motor pins to outputs
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  //turn off motors intitially
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}
void loop(){ 
  directionControl();
  delay(1000);
}
//this lets control spinning direction
void directionControl(){
  analogWrite(enA,255);
  analogWrite(enB,255);
  //one dierction
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  delay(2000);
  //change direction
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  delay(2000);
  //turn off motors
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}
