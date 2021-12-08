int pwmpin=9;
int dirf=7;
int dirb=8;
int enca=12;
int encb=13;
int lenca;
int cenca;//current enc A
int cencb;//current enc B
volatile long int enccount=0;
int target=400;
int error;
int prerror;
float kp=1.2;
int kd;
int k=1;
float raftaar;


void setup() 
{
  Serial.begin(9600);
  pinMode(pwmpin,OUTPUT);
  pinMode(dirf,OUTPUT);
  pinMode(dirb,OUTPUT);
  pinMode(enca,INPUT);
  pinMode(encb,INPUT);
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
           Serial.write("CCW");
           
        }
    
       else//clockwise
        {
            enccount++;
            Serial.write("CW");
        }
    }
    
    lenca=cenca;
   
    
 error=target-enccount;

  raftaar=error*kp ;
  if(raftaar>255)
  {
    raftaar=255;
  }
  if(error>0)
  {
    digitalWrite(dirf,HIGH);
    digitalWrite(dirb,LOW);
  }
  else if(error<0)
  {
    digitalWrite(dirb,HIGH);
    digitalWrite(dirf,LOW);
  }
  if(raftaar<=100)
  {
    k=0;
    analogWrite(pwmpin,0);
    digitalWrite(dirb,LOW);
    digitalWrite(dirf,LOW);
    
  }
  if(error!=0 && k==1)
  {
  analogWrite(pwmpin,abs(raftaar));
 
  }

  if(error==0)
  {
    
    analogWrite(pwmpin,0);
    digitalWrite(dirb,LOW);
    digitalWrite(dirf,LOW);
    
 
  
  }
  
  Serial.print(enccount);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.print(raftaar);
  Serial.print("\n"); 
  
}
