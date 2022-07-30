#define CS A1
#define pwm 4
#define dir1 5
#define dir2 6

#define en1 3
#define en2 7
#define lSwitch 2

#define KP 5
#define KI 2
#define KD 0
int prev_error = 0;

int PID(int currentValue, int targetValue);

volatile long int count = 0;
void setup() {
  Serial.begin(115200);
  pinMode(CS, INPUT);
  pinMode(pwm , OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(en1, INPUT_PULLUP);
  pinMode(en2, INPUT_PULLUP);
  pinMode(lSwitch, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(lSwitch), hom, FALLING);
  attachInterrupt(digitalPinToInterrupt(en1), encoder, RISING);
}

int target = 0, flag = 0, vel, homingFlag = 0;
void loop() {
  Serial.print(analogRead(CS));
  Serial.print("\t");
  Serial.print(count);
  Serial.print("\t");
  if (Serial.available()) {
    char data = Serial.read();
    if (data == '1') {
      target = 1600;
      flag = 1;
    }
    else if (data == '2') {
      target = 0;
      flag = 1;
    }
    else if (data == '8') {
      vel = 255;
      flag = 0;
    }
    else if (data == '9') {
      vel = -255;
      flag = 0;
    }
    else if (data == '5') {
      flag = 0;
      homingFlag = 1;
    }
    else {
      flag = 0;
      vel = 0;
      Serial.println("Stop");
    }
  }

  if (homingFlag == 1) {
    flag = 0;
    vel = -255;
  }
  
  if (flag == 1) {
    vel = PID(count, target);
    if (vel > 255) {
      vel = 255;
    }
    else if (vel < -255) {
      vel = -255;
    }
    if (vel < 120 && vel > 0) {
      vel = 0;
    }
    else if (vel > -120 && vel < 0) {
      vel = 0;
    }
  }
  if(analogRead(CS) > 250){
    vel = 0;
  }
  Serial.println(vel);
  drive(vel);
  // put your main code here, to run repeatedly:
}

void hom() {
  Serial.println("Home Position");
  count = 0;
  homingFlag = 0;
  vel = 0;
}
void encoder() {
  if (digitalRead(en2)) {
    count++;
  }
  else {
    count--;
  }
}

void drive(int vel) {
  if (vel > 0) {
    CW(vel);
  }
  else {
    vel = -vel;
    CCW(vel);
  }
}
void CCW(int vel) {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
  analogWrite(pwm, vel);
}
void CW(int vel) {
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);
  analogWrite(pwm, vel);
}

void Stop() {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  analogWrite(pwm, 0);
}
int PID(int currentValue, int targetValue) {
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error;
  int diff_error = error - prev_error;

  prev_error = error;
  return (KP * prop_error) + (KI * integral_error) + (KD * diff_error);
}

void homing() {
  while (!digitalRead(lSwitch)) {
    Serial.println("Homing");
    CCW(150);
  }
}
