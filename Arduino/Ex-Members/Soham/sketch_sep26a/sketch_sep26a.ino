uint8_t button_pin=D1;
uint8_t led_pin=D5;
uint8_t IRsensor=D2;

void setup() 
{
  // put your setup code here, to run once:

//   pinMode(led_pin,OUTPUT);
//  pinMode(button_pin,INPUT);  
  Serial.begin(9600);
  pinMode(IRsensor,INPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
//  int button_state=digitalRead(button_pin);
//  Serial.print("The button state is:   ");
//  Serial.println(button_state);
// 
  int status=digitalRead(IRsensor);
  Serial.println(status);


  
}
