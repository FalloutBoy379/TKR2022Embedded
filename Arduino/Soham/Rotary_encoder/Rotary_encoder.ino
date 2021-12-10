#define CLK 2
#define DT 3
#define SW 4
int counter = 0;
int currentstateCLK;
int laststateCLK;
string currentdir ="";
unsigned long lastbuttonpress = 0;
void setup()
{
  //set encoder pin as inputs
  pinMode (CLK, INPUT);
  pinMode (DT, INPUT);
  pinMode (SW, INPUT_PULLUP);
  //setup serial monitor
  serial.Begin(9600);
  //read initial state of CLK
  laststateCLK = digitalRead(CLK);
}
void loop()
{
 currentstateCLK = digitalRead(CLK);
 if (currentstateCLK != laststateCLK && currentstateCLK == 1)
 {
  if (digitalRead(DT) != currentstateCLK)
  {
    counter --;
    currentdir = "CCW"; 
  }
  else
  {
    counter ++;
    currentdir = "CW";
  }
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
 }
 laststateCLK = currentstateCLK
 // Read the button state
 int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) 
  {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) 
    {
      Serial.println("Button pressed!");
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  // Put in a slight delay to help debounce the reading
  delay(1);
}
