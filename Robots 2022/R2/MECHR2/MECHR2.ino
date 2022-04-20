#define DIR 8
#define STEP 7
#define SW 4
#define GRIPPER1 22
#define GRIPPER2 23
#define INOUT 24

#define STEPS_PER_REV 6400
#define DISTANCE_PER_REV 42 //mm


#define LAGORI1 0x01
#define LAGORI2 0x02
#define LAGORI3 0x03
#define LAGORI4 0x04
#define LAGORI5 0x05
#define HOMPOS 0xFF

#define LAGORI1POS 0
#define LAGORI2POS 200
#define LAGORI3POS 400
#define LAGORI4POS 600
#define LAGORI5POS 760

int x = 0;
int period = 0;

bool upFlag = 0, downFlag = 0;

byte positionStacking = 0x00;

char data = 0;

#define DIR1 9
#define DIR2 10
#define PWM 11

#define EN1 3
#define EN2 5

#define KP 5
#define KI 2
#define KD 0

#define PISTON 12

int prev_error = 0;


int PID(int currentValue, int targetValue);

//char data;
volatile int count = 0;
int vel = 0;

void setup() {
  Serial2.begin(115200);
  pinMode(EN1, INPUT_PULLUP);
  pinMode(EN2, INPUT_PULLUP);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(PISTON, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(EN1), encoder, RISING);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(SW, INPUT_PULLUP);
  pinMode(GRIPPER1, OUTPUT);
  pinMode(GRIPPER2, OUTPUT);
  pinMode(INOUT, OUTPUT);
  Serial.begin(115200);
}
int distanceTracker = 0;


void loop1() {
  if (Serial.available()) {
    data = Serial.read();
    if (data == '1') {
      for (int i = 0; i < STEPS_PER_REV; i++) {
        moveUp(10);
      }
    }
    else if (data == '0') {
      for (int i = 0; i < STEPS_PER_REV; i++) {
        moveDown(10);
      }
    }
  }
}

void loop() {

  if (Serial.available()) {
    data = Serial.read();
  }
  if (data == '1') {
    positionStacking = LAGORI2;
  }
  else if (data == '2') {
    positionStacking = LAGORI3;
  }
  else if (data == '3') {
    positionStacking = LAGORI4;
  }
  else if (data == '4') {
    positionStacking = LAGORI5;
  }
  else if (data == '0') {
    positionStacking = LAGORI1;
  }
  else if (data == '5') {
    positionStacking = 0x80;
  }
  else if (data == '6') {
    openGripper();
  }
  else if (data == '7') {
    closeGripper();
  }
  else if (data == '8') {
    closeMechanism();
  }
  else if (data == '9') {
    openMechanism();
  }
  else if (data == 'a') {
    vel = 255;
  }
  else if (data == 'b') {
    vel = -255;
  }
  else if (data == 'c') {
    vel = -PID(count, -1600);
  }
  else if (data == 'd') {
    vel = -PID(count, 0);
  }
  else if (data == 'e') {
    digitalWrite(PISTON, HIGH);
  }
  else if (data == 'f') {
    digitalWrite(PISTON, LOW);
  }
  else {
    vel = 0;
  }
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
  drive(vel);

  if (positionStacking == LAGORI2) {
    Serial.println("Moving to Lagori 2");
    moveDistance(LAGORI2POS - distanceTracker);
    distanceTracker += LAGORI2POS - distanceTracker;
    positionStacking = 0x00;
  }
  else if (positionStacking == LAGORI3) {
    Serial.println("Moving to Lagori 3");
    moveDistance(LAGORI3POS - distanceTracker);
    distanceTracker += LAGORI3POS - distanceTracker;
    positionStacking = 0x00;
  }
  else if (positionStacking == LAGORI4) {
    Serial.println("Moving to Lagori 4");
    moveDistance(LAGORI4POS - distanceTracker);
    distanceTracker += LAGORI4POS - distanceTracker;
    positionStacking = 0x00;
  }
  else if (positionStacking == LAGORI5) {
    Serial.println("Moving to Lagori 5");
    moveDistance(LAGORI5POS - distanceTracker);
    distanceTracker += LAGORI5POS - distanceTracker;
    positionStacking = 0x00;
  }
  else if (positionStacking == LAGORI1) {
    Serial.println("Lagori 1");
    moveDistance(LAGORI1POS - distanceTracker);
    distanceTracker = 0;
    positionStacking = 0x00;
  }
  else if (positionStacking == 0x80) {
    //    Serial.println("moving to home");
    while (digitalRead(SW) == HIGH) {
      Serial.println("Not homed");
      moveDistance(-6);
    }
    if (digitalRead(SW) == LOW) {
      Serial.println("homed");
      distanceTracker = 0;
      positionStacking = 0x00;
    }
  }
}

bool moveUp(int vel) {
  //  Serial.println("Moving up");
  digitalWrite(DIR, HIGH);
  digitalWrite(STEP, HIGH);
  delayMicroseconds(vel);
  digitalWrite(STEP, LOW);
  delayMicroseconds(vel);
  return true;
}

bool moveDown(int vel) {
  //  Serial.println("Moving down ");
  digitalWrite(DIR, LOW);
  digitalWrite(STEP, HIGH);
  delayMicroseconds(vel);
  digitalWrite(STEP, LOW);
  delayMicroseconds(vel);
  return true;
}

bool moveDistance(long int dist) {
  Serial.print(dist);
  Serial.print("\t");
  if (dist < 0) {
    dist = -dist;
    long int steps = (STEPS_PER_REV * dist) / DISTANCE_PER_REV;
    Serial.println(steps);
    for (int i = 1; i <= steps; i++) {
      moveDown(10);
    }
  }
  else {
    long int steps = (STEPS_PER_REV * dist) / DISTANCE_PER_REV;
    Serial.println(steps);
    for (int i = 1; i <= steps; i++) {
      moveUp(10);
    }
  }
}

void closeGripper() {
  digitalWrite(GRIPPER1, HIGH);
  digitalWrite(GRIPPER2, LOW);
}

void openGripper() {
  digitalWrite(GRIPPER1, LOW);
  digitalWrite(GRIPPER2, HIGH);
}

void closeMechanism() {
  digitalWrite(INOUT, HIGH);
}

void openMechanism() {
  digitalWrite(INOUT, LOW);
}

void drive(int vel) {
  if (vel > 0) {
    moveUpRack(vel);
  }
  else if (vel < 0) {
    vel = -vel;
    moveDownRack(vel);
  }
  else {
    moveUpRack(0);
  }
}

void moveUpRack(int vel) {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM, vel);
}

void moveDownRack(int vel) {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM, vel);
}

void encoder() {
  if (digitalRead(EN2)) {
    count++;
  }
  else {
    count--;
  }
}


int PID(int currentValue, int targetValue) {
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error;
  int diff_error = error - prev_error;

  prev_error = error;
  return (KP * prop_error) + (KI * integral_error) + (KD * diff_error);
}
