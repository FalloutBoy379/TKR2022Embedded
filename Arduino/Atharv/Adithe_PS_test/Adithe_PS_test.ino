int signal_pin1 =30;
int signal_pin2=31;
int signal_pin3=32;
int button_count=1;
const int pwm = 12 ; 
const int in_1 = 13 ;
const int in_2 = 9 ;
const int in_3 = 22 ;
const int in_4 = 24;
unsigned long curr_millis= millis();
unsigned long  prev_millis;
const long interval1=3000;
const long interval2=500;

int counter = 0;

//Call ps_setup() in void setup()
//Call ps_read() in void loop()



#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))
#define pwm_range 115
#define PS_L1 0
#define PS_R1 1
#define PS_L2 2
#define PS_R2 3
#define PS_L3 4
#define PS_R3 5
#define PS_TRIANGLE 6
#define PS_SQUARE 7
#define PS_CROSS 8
#define PS_CIRCLE 9
#define PS_UP 10
#define PS_LEFT 11
#define PS_DOWN 12
#define PS_RIGHT 13
#define PS_START 14
#define PS_SELECT 15

int prevMillis = 0;
int xj1Offset = 0, yj1Offset = 0, xj2Offset = 0, yj2Offset = 0, calibrateFlag = 0, prevComp = 0;

int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0; //analog values(serially received from remote);
int butt[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //digital values(serially received from remote);

uint8_t RX[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0;
uint8_t TX[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
int flag[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;







bool ps_setup() {
  UBRR3H = BAUDRATE >> 8;
  UBRR3L = BAUDRATE;
  UCSR3B = 0b10011000; //enable RXEN TXEN
  UCSR3C = 0b00000110;
}

ISR(USART3_RX_vect)
{
  RX_count = 1;
  RX_raw = UDR3;
  if ((RX_raw > 200) && (RX_raw < 255))
  {
    RX_ad = RX_raw;
    if ((RX_raw > 230) && (RX_raw < 247))
    {
      uint8_t r_temp0 = (RX_raw - 231);
      butt[r_temp0] = 1;
    }
  }
  else if ((RX_raw >= 0) && (RX_raw < 201))
  {
    uint8_t r_temp1 = (RX_ad - 201);
    if (r_temp1 < 16)
    {
      RX[r_temp1] = RX_raw;
    }
  }
}



//READ DATA FROM CONTROLLER
bool ps_read() {
  RX_count = 0;
  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, pwm_range, (-pwm_range));
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));

  if (butt[PS_START] == 1)
  {
    Serial.println("Start");
    butt[PS_START] = 0;
  }
  if (butt[PS_SELECT] == 1)
  {
    Serial.println("Select");
    butt[PS_SELECT] = 0;
  }
  if (butt[PS_UP] == 1)
  {
    Serial.println("Up");
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    Serial.println("Down");
    butt[PS_DOWN] = 0;
  }
  if (butt[PS_LEFT] == 1)
  {
    Serial.println("left");
    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    Serial.println("right");
    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1)
  {
    button_count++ ;
    Serial.println("square");
    butt[PS_SQUARE] = 0;
  }
  if (butt[PS_CIRCLE] == 1)
  {
    button_count = 0;
    Serial.println("circle");
    butt[PS_CIRCLE] = 0;
  }
  if (butt[PS_TRIANGLE] == 1)
  {
    Serial.println("triangle");
    butt[PS_TRIANGLE] = 0;
  }
  if (butt[PS_CROSS] == 1)
  {
    Serial.println("cross");
    butt[PS_CROSS] = 0;
  }
  if (butt[PS_L1] == 1)
  {

    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {

    Serial.println("R1");
    butt[PS_R1] = 0;
  }
  if (butt[PS_L2] == 1)
  {
    butt[PS_L2] = 0;
  }
  if (butt[PS_R2] == 1)
  {
    butt[PS_R2] = 0;
  }
  if (butt[PS_L3] == 1)
  {
    Serial.println("L3");
    butt[PS_L3] = 0;
  }
  if (butt[PS_R3] == 1)
  {
    Serial.println("R3");
    butt[PS_R3] = 0;
  }
}

void setup()
{
  ps_setup();
  Serial.begin(115200);
  
  pinMode(signal_pin1, OUTPUT);
  pinMode(signal_pin2, OUTPUT);
  pinMode(signal_pin3, OUTPUT);
  pinMode(pwm,OUTPUT) ; 
  pinMode(in_1,OUTPUT) ; 
  pinMode(in_2,OUTPUT) ;
  pinMode(in_3,OUTPUT);
  pinMode(in_4,OUTPUT);
}

void millisfunc1()
{
   if(curr_millis-prev_millis>=interval1) 
  {
    prev_millis= curr_millis;
  }
}

void millisfunc20()
{
   if(curr_millis-prev_millis>=interval2) 
  {
    prev_millis= curr_millis;
  }
}

#define millisfunc2() delay(500)
#define millisfunc1() delay(3000)


void loop()
{
  ps_read();
  if(button_count==2 && counter ==0)
   {
    counter = 1;
   // actuate first piston
   digitalWrite( signal_pin1, HIGH);
   millisfunc2();
   digitalWrite( signal_pin1, LOW);
   digitalWrite( signal_pin2, HIGH);
   digitalWrite(in_1,HIGH) ;
     digitalWrite(in_2,LOW) ;
     analogWrite(pwm,255) ;
    millisfunc1(); 
     digitalWrite( signal_pin2,LOW);     
     digitalWrite(in_1,LOW) ;
     digitalWrite(in_2,LOW) ;
   }
   
   if (button_count==3 && counter ==1)
    {
      counter =2;    
     digitalWrite( signal_pin1, HIGH);
     millisfunc2();
     digitalWrite( signal_pin1, LOW);
     digitalWrite( signal_pin3, HIGH);    
     digitalWrite(in_3,LOW) ;
     digitalWrite(in_4,HIGH) ;
     millisfunc1();
     digitalWrite( signal_pin3, LOW);
     digitalWrite(in_3,LOW) ;
     digitalWrite(in_4,LOW) ; 
    }  
  



   if(button_count==4 && counter ==2)
   {
     digitalWrite( signal_pin1, HIGH);
     millisfunc2();
     digitalWrite( signal_pin1, LOW);  
   }
   Serial.println(button_count);
}
