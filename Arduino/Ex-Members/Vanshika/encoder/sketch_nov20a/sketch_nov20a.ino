#define inputCLK 4  // initializing
#define inputDT 5
int counter=0; // this is the value encoder will be changing.
int currentstateclk; //to know wether the encoder has moved or not we need current state and previous state
int previousstateclk;
String encidr=" "; // will show the direction of encoder
void setup() {
  // setting encoder pins as inputs
  pinMode(inputCLK,INPUT);
 pinMode(inputDT,INPUT);
 // setting serial monitor
 Serial.begin(9600);
// assigning previous value of encoder.
previousstateclk=digitalRead(inputCLK);
}

void loop() {
  // assigning the current state of clk
  currentstateclk=digitalRead(inputCLK);
  // if the previous state and current state is different thus a pulse has occured
  if(currentstateclk!=previousstateclk)
  {
    // if the input clk is diff from input dt then encoder rotates anticlockwise
    if(digitalRead(inputDT)!=currentstateclk)
    {
      counter--;
      encidr="ANTICLOCKWISE";
    }
    else
    {
      counter++;
      encidr="CLOCKWISE";
    }
    Serial.print("Direction: ");
    Serial.print(encidr);
    Serial.print("--Value: ");
    Serial.println(counter);
  }
// update the previous state with  current state 
previousstateclk=currentstateclk; 
  
}
