int pwmpin=9;
int dirf=7;
int dirb=8;
int enca=12;
int encb=13;
int lenca;
int cenca;//current enc A
int cencb;//current enc B
int enccount=0;
int target=50;
int error;
int prerror;
float kp=0.7;
int kd;
int k=1;
float speed_bot;
int posi;
int pos=0;

// void readEncoder(){
//   int b = digitalRead(encb);
//   if(b > 0){
//     posi++;
//   }
//   else{
//     posi--;
//   }
// }

void setup() 
{
  Serial.begin(9600);
  pinMode(pwmpin,OUTPUT);
  pinMode(dirf,OUTPUT);
  pinMode(dirb,OUTPUT);
  pinMode(enca,INPUT);
  pinMode(encb,INPUT);
  // attachInterrupt(digitalPinToInterrupt(enca),readEncoder,RISING);
  lenca=digitalRead(enca);
  
}

void loop()
{
      cenca = digitalRead(enca);
    cencb = digitalRead(encb);    
   
  
     //pid();
  
    if(lenca != cenca && cenca == 1)
    {
        if(digitalRead(encb) != cenca)//anti-clockwise
        {
           enccount--; 
//           Serial.write("CCW");
           
        }
    
       else//clockwise
        {
            enccount++;
//            Serial.write("CW");
        }
    }
    
    lenca=cenca;
    
  // pos = 0; 
  // noInterrupts(); // disable interrupts temporarily while reading
  // pos = posi;
  // interrupts(); //     
 error=target-enccount;
 if(k==0)
 {
  speed_bot=error*kp ;
  if(speed_bot>255)
  {
    speed_bot=255;
  }
  if(error>0)
  {
    digitalWrite(dirf,HIGH);
    digitalWrite(dirb,LOW);
  }
  else if(error<0)
  {
    digitalWrite(dirf,LOW);
    digitalWrite(dirb,HIGH);
  }
  if(speed_bot<=100)
  {
    k=0;
    analogWrite(pwmpin,0);
    digitalWrite(dirf,LOW);
    digitalWrite(dirb,LOW);
  }
  if(speed_bot!=0 && k==1)
  {
  analogWrite(pwmpin,fabs(speed_bot));
  }
 }
  if(speed_bot==0)
  {
    k=0;
//    Serial.println("GG");
    analogWrite(pwmpin,LOW);
    digitalWrite(dirb,LOW);
    digitalWrite(dirf,LOW);
    digitalWrite(enca,LOW);
    digitalWrite(encb,LOW);
 
  
  }
  
  Serial.print(enccount);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.print(speed_bot);
  Serial.print("\n"); 
  
}
