void setup() {
  Serial2.begin(115200);
  Serial.begin(115200);// put your setup code here, to run once:

}

int ints = 0;
int count = 0;
int encoder1 = 0;
int potValue = 0;
void loop() {

//  Serial.println(encoder1/(3.6*4.1));
//  Serial.println("\t");
//  Serial.println(potValue);
  //  Serial.print(count);
  //  Serial.print("\t");
  //  Serial.println(ints);
  if (Serial2.available()) 
  {
  Serial.print(encoder1/(3.6*4.1));
  Serial.print("\t");
  Serial.println(potValue);
    char data = Serial2.read();
    if (data == '_') {
      int data_high = readSerial2();
      int data_low = readSerial2();
      count = data_high << 8  | data_low;

    }
    else if (data == '|') {
      int data_high = readSerial2();
      int data_low = readSerial2();
      ints = data_high << 8  | data_low;
    }
    else if (data == '#'){
      int data_high = readSerial2();
      int data_low = readSerial2();
      potValue = data_high << 4  | data_low;
    }

    encoder1 = (1024 * ints) + count;
  }
  // put your main code here, to run repeatedly:

}

uint8_t readSerial2() {
  while (!Serial2.available());
  return Serial2.read();
}
