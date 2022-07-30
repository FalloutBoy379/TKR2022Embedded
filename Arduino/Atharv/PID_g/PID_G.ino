int pwmpin=9;
int dirf=8;
int dirb=7;
int enca=2;
int encb=3;
int lenca;
int cenca;//current enc A
int cencb;//current enc B
volatile long int enccount=0;
int target=100;
int error;
int prerror;
double kp=9;
double ki;
int kd;
int k=1;
float movement;


void setup() 
{
  Serial.begin(9600);
  pinMode(pwmpin,OUTPUT);
  pinMode(dirf,OUTPUT);
  pinMode(dirb,OUTPUT);
  pinMode(enca,INPUT);
  pinMode(encb,INPUT);++
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

  movement=error*kp ;
  if(movement>255)
  {
    movement=255;
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
  if(movement<=30)
  {
    k=0;
    analogWrite(pwmpin,0);
    digitalWrite(dirb,LOW);
    digitalWrite(dirf,LOW);
    
  }
  if(error!=0 && k==1)
  {
  analogWrite(pwmpin,abs(movement));
 
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
  Serial.print(movement);
  Serial.print("\n"); 
  
}
