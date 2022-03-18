uint8_t led_pin=D5;
uint8_t button_pin=D1;
int led_state=LOW;

void blink_LED(int pin,int delayTime)
{
digitalWrite(pin,HIGH);
delay(delayTime);
digitalWrite(pin,LOW);
delay(delayTime);
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(led_pin,OUTPUT);
  pinMode(button_pin,INPUT);  
}

void loop() 
{
  // put your main code here, to run repeatedly:
//  digitalWrite(led_pin,HIGH);
//  delay(500);
//  digitalWrite(led_pin,LOW);
//  delay(500);


//  int button_state=digitalRead(button_pin);
//  digitalWrite(led_pin,button_state);

int button_state=digitalRead(button_pin);
if(button_state==HIGH)
{
blink_LED(led_pin,1000);
}
}
