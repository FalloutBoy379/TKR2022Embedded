#define Motor1Pin1 D5
#define Motor1Pin2 D6

#define Motor2Pin1 D0
#define Motor2Pin2 D1

#define Sensor1 D2
#define Sensor2 D7


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
void drive(char dir);

char auth[] = "PGM_08JRYPDMirvUJcjdp1uhjUB4Wn-E";
char ssid[] = "Galaxy A21s5781";
char pass[] = "ghostly842";

void setup()
{
  // Debug console
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);

  pinMode(Motor1Pin1, OUTPUT);
  pinMode(Motor1Pin2, OUTPUT);
  pinMode(Motor2Pin1, OUTPUT);
  pinMode(Motor2Pin2, OUTPUT);

  digitalWrite(Motor1Pin1, LOW);
  digitalWrite(Motor1Pin2, LOW);
  digitalWrite(Motor2Pin1, LOW);
  digitalWrite(Motor2Pin2, LOW);
}

void loop()
{
//  int a = digitalRead(Sensor1);
//  int b = digitalRead(Sensor2);
//  Serial.print(a);
//  Serial.print("\t");
//  Serial.println(b);
//  if(a == 1 && b == 0){
//    drive('l');
//  }
//  else if(a == 0 && b == 1){
//    drive('r');
//  }
//  else if(a == 0 && b == 0){ 
//    drive('b');
//  
//  }
//  else {
//    drive('s');
//  }
  Blynk.run();
}

void drive(char dir){
  if(dir == 'f' || dir == 'F'){
    digitalWrite(Motor1Pin1, HIGH);
    digitalWrite(Motor1Pin2, LOW);
    digitalWrite(Motor2Pin1, HIGH);
    digitalWrite(Motor2Pin2, LOW);
    Serial.println("Forward");
  }
  else if(dir == 'b' || dir=='B'){
    digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor1Pin2, HIGH);
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor2Pin2, HIGH);
    Serial.println("Backward");
  }
  else if(dir == 'r' || dir == 'R'){
    digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor1Pin2, HIGH);
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor2Pin2, LOW);
    Serial.println("Right");
  }
  else if(dir == 'l' || dir == 'L'){
    digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor1Pin2, LOW);
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor2Pin2, HIGH);
    Serial.println("Left");
  }
  else{
    digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor1Pin2, LOW);
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor2Pin2, LOW);
    Serial.println("Stop");
  }
}

BLYNK_WRITE(V0){
  int x = param[0].asInt();
  int y = param[1].asInt();
  if (x>=5 && y<5 && y>-5){
    drive('b');
  }
  else if(x<=-5 && y<5 && y>-5){
    drive('f');
  }
  else if(y<=-5){
    drive('l');
  }
  else if(y>=5){
    drive('r');
  }
  else{
    drive('stop');
  }
  
  Serial.println(x);
  //Serial.println(y);
  //Serial.println(z);
}
