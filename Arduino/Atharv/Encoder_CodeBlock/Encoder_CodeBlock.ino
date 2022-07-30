#define EN1A 2
#define EN1B 4

volatile int count;

void setup() {
  Serial.begin(115200);
  pinMode(EN1A, INPUT_PULLUP);
  pinMode(EN1B, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(EN1A), encoder, RISING);

}

void loop() {
//  Serial.println(count);
  Serial.print(digitalRead(EN1A));
  Serial.print("\r\n");
  Serial.print(digitalRead(EN1B));
}


void encoder(){
  if(digitalRead(EN1A) == HIGH){
    count++;
  }
  else{
    count--;
  }
}
