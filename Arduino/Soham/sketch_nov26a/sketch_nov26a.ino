char intswitch[10];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.readBytes(intswitch,5);//Read the serial data and store in var
  Serial.print(intswitch);//Print the data on serial monitor
  delay(1000);
}
