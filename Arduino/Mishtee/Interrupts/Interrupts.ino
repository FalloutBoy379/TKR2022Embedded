//Set up the button
int buttonInt = 0; //pin2
//setup LEDs
int yellowLed = 11;
int redLed = 10;
int greenLed = 9;
int nullLed = 6; //to lock brightness in leds
volatile int selectedLed = greenLed;
//volatile as the value is changing
//set up sensor
int distPin = A0;
void setup ()
{
  //set pin mode
  pinMode(redLed,OUTPUT);
  pinMode(yellowLed,OUTPUT);
  pinMode(greenLed,OUTPUT);
  pinMode(nullLed,OUTPUT);
  //attach interrupt
  attachInterrupt (buttonInt,swap,RISING);
}
//interrupt function
void swap()
{
  if (selectedLed == greenLed)
    selectedLed = redLed;
  else if (selectedLed == redLed)
    selectedLed = yellowLed;
    else if (selectedLed == yellowLed)
    selectedLed = nullLed;
    else 
    selectedLed = greenLed;
}
void loop()
{
  //read dist sensor
  int dist = analogRead(distPin);
  int brightness = map (dist,0,1023,0,255);
  //control led brightness
  analogWrite(selectedLed,brightness);
}
