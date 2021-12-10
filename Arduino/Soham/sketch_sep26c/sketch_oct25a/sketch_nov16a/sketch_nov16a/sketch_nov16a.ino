int enPin1=2;
int enPin2=3;
volatile int lastEnc=0;
volatile int encValue=0;

long lastencValue=0;

int lastMSB=0;
int lastLSB=0;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
 pinMode(enPin1,INPUT_PULLUP);
  pinMode(enPin2,INPUT_PULLUP);

  digitalWrite(enPin1,HIGH);
  digitalWrite(enPin2,HIGH);

  attachInterrupt(0,updateEncoder,CHANGE);
  attachInterrupt(1,updateEncoder,CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(encValue);
}

void updateEncoder()
{
  int MSB=digitalRead(encPin1);
  int LSB=digitalRead(encPin2);
    
  
}
