#include <PS5BT.h>
#include <usbhub.h>
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
//#define
/* You can create the instance of the PS5BT class in two ways */
// This will start an inquiry and then pair with the PS5 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS5 controller will then start to blink rapidly indicating that it is in pairing mode
PS5BT PS5(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
//PS5BT PS5(&Btd);

bool printAngle = false, printTouch = false;
uint16_t lastMessageCounter = -1;
uint8_t player_led_mask = 0;
bool microphone_led = false;
uint32_t ps_timer;

int xj1, yj1, xj2, yj2, r2, l2;
int counter = 0;
char data = '0';

void setup() {
  Serial.begin(9600);
  if (Usb.Init() == -1) {
    while (1); // Halt
  }
}

void loop() {
  Usb.Task();

  if (PS5.connected() && lastMessageCounter != PS5.getMessageCounter()) {
    if(Serial.available()){
      data = Serial.read();
    }

    if(data == '0'){
      PS5.setLed(255, 0, 174);
    }
    else if(data == '1'){
      PS5.setLed(255, 40, 40);
    }
    if (counter == 0) {
      counter = 1;
      PS5.setPlayerLed(0b00001010);
    }
//    PS5.setLed(255, 0, 174);
    //    PS5.setLed(20, 255, 20);
    lastMessageCounter = PS5.getMessageCounter();
    xj1 = PS5.getAnalogHat(LeftHatX);
    yj1 = PS5.getAnalogHat(LeftHatY);
    xj2 = PS5.getAnalogHat(RightHatX);
    yj2 = PS5.getAnalogHat(RightHatY);
    r2 = PS5.getAnalogButton(R2);
    l2 = PS5.getAnalogButton(L2);


    xj1 = map(xj1, 0, 255, 0, 200);
    yj1 = map(yj1, 0, 255, 0, 200);
    xj2 = map(xj2, 0, 255, 0, 200);
    yj2 = map(yj2, 0, 255, 0, 200);
    r2 = map(r2, 0, 255, 0, 200);
    l2 = map(l2, 0, 255, 0, 200);

    Serial.write(201);
    Serial.write(xj1);
    Usb.Task();
    delay_ms(10);
    Serial.write(202);
    Serial.write(yj1);
    Usb.Task();
    delay_ms(10);
    Serial.write(203);
    Serial.write(xj2);
    Usb.Task();
    delay_ms(10);
    Serial.write(204);
    Serial.write(yj2);
    delay_ms(10);
    Serial.write(205);
    Serial.write(r2);
    delay_ms(10);
    Serial.write(206);
    Serial.write(l2);
    delay_ms(10);
    Usb.Task();

    if (PS5.getButtonClick(RIGHT)) Serial.write(244);
    Usb.Task();
    if (PS5.getButtonClick(DOWN)) Serial.write(243);
    Usb.Task();
    if (PS5.getButtonClick(LEFT)) Serial.write(242);
    Usb.Task();
    if (PS5.getButtonClick(UP)) Serial.write(241);
    Usb.Task();
    if (PS5.getButtonClick(SQUARE)) Serial.write(238);
    Usb.Task();
    if (PS5.getButtonClick(CROSS)) Serial.write(239);
    Usb.Task();
    if (PS5.getButtonClick(CIRCLE)) Serial.write(240);
    Usb.Task();
    if (PS5.getButtonClick(TRIANGLE)) Serial.write(237);
    Usb.Task();
    if (PS5.getButtonClick(L1)) Serial.write(231);
    Usb.Task();
    if (PS5.getButtonClick(R1)) Serial.write(232);
    Usb.Task();
    if (PS5.getButtonClick(MICROPHONE)) Serial.write(233);
    Usb.Task();
    if (PS5.getButtonClick(SHARE)) Serial.write(246);
    Usb.Task();
    if (PS5.getButtonClick(OPTIONS)) Serial.write(245);
    Usb.Task();
    if (PS5.getButtonClick(PS)) Serial.write(236);
    Usb.Task();
    if (PS5.getButtonClick(R3)) Serial.write(247);
    Usb.Task();
    if (PS5.getButtonClick(L3)) Serial.write(248);
    Usb.Task();
    if(PS5.getButtonClick(TOUCHPAD)) Serial.write(249);
    Usb.Task();
  }
}

void delay_ms(int ms) {
  long int   currentTime = millis();
  while (millis() - currentTime < ms) {
    Usb.Task();
  }
}
