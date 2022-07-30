int analpin = A0; 
int analval,anal1,anal2; 

int button=4;
int pwmpin=9;
int dirf=7;
int dirb=8;

int error=0;
int target=200;

float kp=77;
float raftaar=0.0;

bool buttonState = false;

void setup()
{
  //motor pins:2 for direction and 1 pwm
  pinMode(pwmpin,OUTPUT);
  pinMode(dirf,OUTPUT);
  pinMode(dirb,OUTPUT);

  //pin to read analog values from KY 024
  pinMode(analpin, INPUT); 
  pinMode(button,INPUT);
  
  Serial.begin(9600);
  
  anal1 = analogRead(analpin);
}

void loop ()
{
  buttonState = digitalRead(button);
  
  anal2 = analogRead(analpin);
  analval= anal2-anal1;

  if(target>=analval)
  {
  error = target - analval;
  }
  else if(analval>target)
  {
    error = analval - target;
  }
  
  raftaar = kp*error;
  
  if(raftaar>1000)
  {
    raftaar=1000;
  }
  
  if(buttonState==true)
 {
    digitalWrite(dirf,LOW);
    digitalWrite(dirb,HIGH);
 } 
  else if(buttonState==false)
  {    
    digitalWrite(dirb,LOW);
    digitalWrite(dirf,HIGH);  
  }
  else
  {
    
  }
    analogWrite(pwmpin,abs(raftaar));
    
    Serial.print(raftaar);
  Serial.print("\t");
    Serial.print("\t");
  Serial.print(analval);
    Serial.print("\t");
  Serial.print(error);
  Serial.print("\n");

  anal1=anal2;
}
