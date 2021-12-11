/*#include<Wire.h>
void setup()
{
  Wire.begin(8);
  Wire.onReceive(recieveEvent);
  Serial.begin(9600);
}
void loop()
{
  delay(100);
}
void recieveEvent(int howMany)
{
  while(1<Wire.available())
  {
    char c = Wire.read();
    Serial.print(c);
  }
  int x = Wire.read();
  Serial.println(x);
}*/
#include <Wire.h>

void setup() {
 Wire.begin(8);                /* join i2c bus with address 8 */
 Wire.onReceive(receiveEvent); /* register receive event */
 Wire.onRequest(requestEvent); /* register request event */
 Serial.begin(9600);           /* start serial comm. */
 Serial.println("I am I2C Slave");
}

void loop() {
 delay(100);
}

// function that executes whenever data is received from master
void receiveEvent(int howMany) {
 while (0 <Wire.available()) {
    char c = Wire.read();      /* receive byte as a character */
    Serial.print(c);           /* print the character */
  }
 Serial.println();             /* to newline */
}

// function that executes whenever data is requested from master
void requestEvent() {
 Wire.write("Hi Master");  /*send string on request */
}
