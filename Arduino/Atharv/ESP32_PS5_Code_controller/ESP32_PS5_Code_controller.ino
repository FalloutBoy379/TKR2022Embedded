#include <ps5Controller.h>

#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
//  ps5.begin("08:25:25:be:d6:f5"); //replace with your MAC address //Ansh
//  ps5.begin("0c:ec:8d:49:48:bb");//Monel
//  ps5.begin("58:85:a2:30:29:34"); //Pratik
  ps5.begin("48:74:12:44:20:0c");// Atharv
//  ps5.begin("D8:DC:40:13:32:BD");// Gautam
//ps5.begin("44:46:87:f4:01:b6"); //Milind
//ps5.begin("98:B8:BC:D3:60:BB"); //Adithe
  Serial.println("Ready.");
}

int xj1, xj2, yj1, yj2;

void loop() {
  //  while (ps5.isConnected() == false) { // commented out as ps5 controller seems to connect quicker when microcontroller is doing nothing
  //    Serial.println("PS5 controller not found");
  //    delay(300);
  //  }
  while (ps5.isConnected() == true) {

    xj1 = ps5.LStickX();
    yj1 = ps5.LStickY();
    xj2 = ps5.RStickX();
    yj2 = ps5.RStickY();

    xj1 = map(xj1, -128, 128, 0, 200);
    yj1 = map(yj1, -128, 128, 0, 200);
    xj2 = map(xj2, -128, 128, 0, 200);
    yj2 = map(yj2, -128, 128, 0, 200);
   
    Serial2.write(201);
    Serial2.write(xj1);
    delay(10);
    Serial2.write(202);
    Serial2.write(yj1);
    delay(10);

    Serial2.write(203);
    Serial2.write(xj2);
    delay(10);

    Serial2.write(204);
    Serial2.write(yj2);
    delay(10);
   
   
    if (ps5.Right()) Serial2.write(244);
    if (ps5.Down()) Serial2.write(243);
    if (ps5.Up()) Serial2.write(241);
    if (ps5.Left()) Serial2.write(242);

    if (ps5.Square()) Serial2.write(238);
    if (ps5.Cross()) Serial2.write(239);
    if (ps5.Circle()) Serial2.write(240);
    if (ps5.Triangle()) Serial2.write(237);

//    if (ps5.UpRight()) Serial.println("Up Right");
//    if (ps5.DownRight()) Serial.println("Down Right");
//    if (ps5.UpLeft()) Serial.println("Up Left");
//    if (ps5.DownLeft()) Serial.println("Down Left");

    if (ps5.L1()) Serial2.write(231);
    if (ps5.R1()) Serial2.write(232);

    if (ps5.Share()) Serial2.write(246);
    if (ps5.Options()) Serial2.write(245);
//    if (ps5.L3()) Serial2.write("L3 Button");
//    if (ps5.R3()) Serial.println("R3 Button");

    if (ps5.PSButton()) Serial2.write(236);
    if (ps5.Touchpad()) Serial2.write(235);

    if (ps5.L2()) {
      Serial.printf("L2 button at %d\n", ps5.L2Value());
    }
    if (ps5.R2()) {
      Serial.printf("R2 button at %d\n", ps5.R2Value());
    }

    if (ps5.LStickX()) {
      Serial.printf("Left Stick x at %d\n", ps5.LStickX());
    }
    if (ps5.LStickY()) {
      Serial.printf("Left Stick y at %d\n", ps5.LStickY());
    }
    if (ps5.RStickX()) {
      Serial.printf("Right Stick x at %d\n", ps5.RStickX());
    }
    if (ps5.RStickY()) {
      Serial.printf("Right Stick y at %d\n", ps5.RStickY());
    }

    //    if (ps5.Charging()) Serial.println("The controller is charging"); //doesn't work
    //    if (ps5.Audio()) Serial.println("The controller has headphones attached"); //doesn't work
    //    if (ps5.Mic()) Serial.println("The controller has a mic attached"); //doesn't work

    //    Serial.printf("Battery Level : %d\n", ps5.Battery()); //doesn't work

    Serial.println();
    // This delay is to make the output more human readable
    // Remove it when you're not trying to see the output
    //    delay(300);
  }
}
