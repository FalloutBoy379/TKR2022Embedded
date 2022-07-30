int pot = 4;
int potVal = 0;

float weight = 0.5;
int potFilterVal = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(pot,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  potVal = analogRead(pot);
  potFilterVal = potVal*(weight) + (1-weight)*potFilterVal;
  
  Serial.println(potFilterVal);
//  Serial.print("\t");
//  Serial.println(potVal);
}
