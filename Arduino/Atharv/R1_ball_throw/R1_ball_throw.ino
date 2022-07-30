  // pwm 5 40,41
  // pwm 6 43,44
  
  int upPin = 5;
  int dpPin = 6;
  int dForward = 43;
  int dBackward = 44;
  int uForward = 41;
  int uBackward = 40;
  
  void setup()
  {
    pinMode(upPin,OUTPUT);
    pinMode(dpPin,OUTPUT);
    pinMode(dForward,OUTPUT);
    pinMode(dBackward,OUTPUT);
    pinMode(uForward,OUTPUT);
    pinMode(uBackward,OUTPUT);
    Serial.begin(9600);
  }
  
  void loop()
  {
    while(Serial.available() == 0);
  
    char mode = Serial.read();
    int i;
    if(mode =='1')
    {
      Serial.println("on");
     digitalWrite(uForward,HIGH);
     digitalWrite(uBackward,LOW);
     i = 0;
     while( i<255)
     {
      analogWrite(upPin ,i);
      i = i+ 10;
     }
  
     
     digitalWrite(dForward,LOW);
     digitalWrite(dBackward,HIGH);
     i = 0;
     while( i<255)
     {
      analogWrite(dpPin ,i);
      i = i+ 10;
     }
    }
  
  
    else if(mode =='0')
    {
     Serial.println("off");
     digitalWrite(uForward,HIGH);
     digitalWrite(uBackward,LOW);
     i = 255;
     while( i>0 )
     {
      analogWrite(upPin ,i);
      i = i- 10;
     }
     digitalWrite(upPin, LOW);
  
     
     digitalWrite(dForward,LOW);
     digitalWrite(dBackward,HIGH);
     i = 255;
     while( i>0 )
     {
      analogWrite(dpPin ,i);
      i = i- 10;
     }
     digitalWrite(dpPin,LOW);
    }
  
    else
    {
      Serial.println("invalid");
    }
  
    
  }
