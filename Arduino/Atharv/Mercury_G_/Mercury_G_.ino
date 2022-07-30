
//Channel A is outer and Channel B is inner

/*
  //         //
**1       2**
  //_________//
  //_________//
  //_________//
  //_________//
  //_________//
  //_________//
**4       3**
  //         //
*/
#define MOSFET1 8  //Connected to STM 10B from bottom //2&4 DOWN/UP
#define MOSFET2 9  //Connected to STM 10A from bottom //3&1 DOWN/UP
#define MOSFET3 10  //Connected to STM 11B from bottom // BACK
#define MOSFET4 11   //Connected to STM 11A from bottom // FRONT

void setup() {
  pinMode(MOSFET1, OUTPUT);
  pinMode(MOSFET2, OUTPUT);
  pinMode(MOSFET3, OUTPUT);
  pinMode(MOSFET4, OUTPUT);
  Serial.begin(115200);
  // put your setup code here, to run once:

}
char data;
void loop() {
  if (Serial.available()) {
    char new_data = Serial.read();
    if (data != new_data) {
      data = new_data;
    }
    if (data == '1') {
      for(int i = 0; i <= 10; i++) {
        digitalWrite(MOSFET3, HIGH);
        digitalWrite(MOSFET4, LOW);
        delay(200);
        digitalWrite(MOSFET1, LOW);
        digitalWrite(MOSFET2, LOW);
        delay(200);
        digitalWrite(MOSFET1, HIGH);
        digitalWrite(MOSFET2, HIGH);
        Serial.println("1");
      }

    }
    else if (data == '2') {
      Serial.println("2");
      for (int i = 0; i <= 10; i++) {
        digitalWrite(MOSFET3, LOW);
        digitalWrite(MOSFET4, HIGH);
        delay(200);
        digitalWrite(MOSFET1, LOW);
        digitalWrite(MOSFET2, LOW);
        delay(200);
        digitalWrite(MOSFET1, HIGH);
        digitalWrite(MOSFET2, HIGH);
        Serial.println("1");
      }
    }
    else if (data == '0') {

    }
  }

  //  digitalWrite(MOSFET3, HIGH);
  //  digitalWrite(MOSFET4, HIGH);
  //  digitalWrite(MOSFET1, HIGH);
  //  digitalWrite(MOSFET2, HIGH);
  //  delay(550);
  //  digitalWrite(MOSFET3, LOW);
  //  digitalWrite(MOSFET4, LOW);
  //  digitalWrite(MOSFET1, HIGH);
  //  digitalWrite(MOSFET2, HIGH);
  //  delay(550);
}
//  if(Serial.available()){
//    char data = Serial.read();
//    if(data == '1'){
//      digitalWrite(MOSFET3, HIGH);
//      digitalWrite(MOSFET4, HIGH);
//      digitalWrite(MOSFET1, HIGH);
//      digitalWrite(MOSFET2, HIGH);
//    }
//    else if(data == '2'){
//      digitalWrite(MOSFET3, LOW);
//      digitalWrite(MOSFET4, LOW);
//      digitalWrite(MOSFET1, HIGH);
//      digitalWrite(MOSFET2, HIGH);
//    }
//    if(data == '1'){
//      Serial.println("Mosfet 1 HIGH");
//      digitalWrite(MOSFET1, HIGH);
//      digitalWrite(MOSFET2, LOW);
//      digitalWrite(MOSFET3, LOW);
//      digitalWrite(MOSFET4, LOW);
//    }
//    else if(data == '2'){
//      Serial.println("Mosfet 2 HIGH");
//      digitalWrite(MOSFET1, LOW);
//      digitalWrite(MOSFET2, HIGH);
//      digitalWrite(MOSFET3, LOW);
//      digitalWrite(MOSFET4, LOW);
//    }
//    else if(data == '3'){
//      Serial.println("Mosfet 3 HIGH");
//      digitalWrite(MOSFET1, LOW);
//      digitalWrite(MOSFET2, LOW);
//      digitalWrite(MOSFET3, HIGH);
//      digitalWrite(MOSFET4, LOW);
//    }
//    else if(data == '4'){
//      Serial.println("Mosfet 4 HIGH");
//      digitalWrite(MOSFET1, LOW);
//      digitalWrite(MOSFET2, LOW);
//      digitalWrite(MOSFET3, LOW);
//      digitalWrite(MOSFET4, HIGH);
//    }
//}
// put your main code here, to run repeatedly:

//}
