//------------------------------------------- Stepper motor pins definition -------------------------------------------------------

#define DIR 5
#define STEP 6
#define SW 2
#define GRIPPER1 22   //PISTONS USED TO OPEN AND CLOSE LAGORI GRIPPERS
#define GRIPPER2 23   
#define INOUT 24      // PISTON USED TO OPEN AND CLOSE STACKING MECHANISM

///////// Others ///////////////
#define STEPS_PER_REV 6400
#define DISTANCE_PER_REV 42 //mm

//-------------------------------------------------------- LAGORI DEFINITIONS ----------------------------------------------------------

#define LAGORI1 0x01
#define LAGORI2 0x02
#define LAGORI3 0x03
#define LAGORI4 0x04
#define LAGORI5 0x05
#define PICKPOSI 0x06
#define HOMPOS 0xFF

#define LAGORI1POS 4
#define LAGORI2POS 90
#define LAGORI3POS 280
#define LAGORI4POS 470
#define LAGORI5POS 650
#define PICKPOS    770

//////////OTHERS//////////
int x = 0;
int period = 0;

bool upFlag = 0, downFlag = 0;

byte positionStacking = 0x00;

char data = 0;

//-------------------------------------------- BALL PICKING DEFINITIONS ------------------------------------------

#define DIR1 9
#define DIR2 10
#define PWM 11

#define EN1 3
#define EN2 5

#define KP 5
#define KI 2
#define KD 0

#define PISTON 12      // PISTON USED TO OPEN AND CLOSE BALL PICKING MECHANISM

///////OTHERS////////////////
int prev_error = 0;


int PID(int currentValue, int targetValue);

//char data;
volatile int count = 0;
int vel = 0;

//-------------------------------------------------- VOID SETUP --------------------------------------------------------

void setup() {
  //Serial.begin(115200);
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

//------------------------------------------------- USE THIS ONLY TO CHECK WORKING OF THE MOTOR ... ELSE COMMENT OUT --------------------------------
/*void loop1() {
  if (Serial.available())
  {
    data = Serial.read();
  }
  if (data == '1') {
   moveDistance(10);
  }
  else if (data == '0') {
    for (int i = 0; i < STEPS_PER_REV; i++) 
    {
      moveDown(10);
    }
  }
  else if (data == '2') 
  {
    //    Serial.println("moving to home");
    while (digitalRead(SW) == HIGH) {
      Serial.print("Not homed");
      moveDistance(-10);
    }
    if (digitalRead(SW) == LOW) {
      Serial.print("homed");
      distanceTracker = 0;
      positionStacking = 0x00;
    }
  }

}*/

//--------------------------------------------- VOID LOOP --------------------------------------------

void loop() {

  if (Serial.available()) {
    data = Serial.read();
  }
  if (data == '1') {
    positionStacking = LAGORI1;
  }
  else if (data == '2') {
    positionStacking = LAGORI2;
  }
  else if (data == '3') {
    positionStacking = LAGORI3;
  }
  else if (data == '4') {
    positionStacking = LAGORI4;
  }
  else if (data == '5') {
    positionStacking = LAGORI5;
  }
  else if (data == '0') {
    positionStacking = 0x80;
  }
  else if (data == '7') {
    openGripper();
  }
  else if (data == '6') {
    closeGripper();
  }
  else if (data == '9') {
    closeMechanism();
  }
  else if (data == '8') {
    openMechanism();
  }
  else if(data == 'u'){
    moveDistance(10);
  }
  else if(data == 'd'){
    moveDistance(-10);
  }
  else if(data == 'P'){
    positionStacking = PICKPOSI;
  }
  Serial.print(distanceTracker);
  Serial.print("\t");
  if (positionStacking == LAGORI2) {
    Serial.print("Moving to Lagori 2");
    Serial.print("\t");
    moveDistance(LAGORI2POS - distanceTracker);
    Serial.print(LAGORI2POS - distanceTracker);
    Serial.print("\t");
    distanceTracker += LAGORI2POS - distanceTracker;
    positionStacking = 0x00;
  }
  else if (positionStacking == LAGORI3) {
    Serial.print("Moving to Lagori 3");
    Serial.print("\t");
    moveDistance(LAGORI3POS - distanceTracker);
    Serial.print(LAGORI3POS - distanceTracker);
    Serial.print("\t");
    distanceTracker += LAGORI3POS - distanceTracker;
    positionStacking = 0x00;
  }
  else if (positionStacking == LAGORI4) {
    Serial.print("Moving to Lagori 4");
    Serial.print("\t");
    moveDistance(LAGORI4POS - distanceTracker);
    Serial.print(LAGORI4POS - distanceTracker);
    Serial.print("\t");
    distanceTracker += LAGORI4POS - distanceTracker;
    positionStacking = 0x00;
  }
  else if (positionStacking == LAGORI5) {
    Serial.print("Moving to Lagori 5");
    Serial.print("\t");
    Serial.print(LAGORI5POS - distanceTracker);
    Serial.print("\t");
    moveDistance(LAGORI5POS - distanceTracker);
    distanceTracker += LAGORI5POS - distanceTracker;
    positionStacking = 0x00;
  }
  else if (positionStacking == LAGORI1) {
    Serial.print("Lagori 1");
    Serial.print("\t");
    moveDistance(LAGORI1POS - distanceTracker);
    distanceTracker = 0;
    positionStacking = 0x00;
  }
  else if (positionStacking == 0x80) {
        Serial.println("moving to home");
    while (digitalRead(SW) == HIGH) {
      Serial.print("Not homed");
      moveDistance(-6);
    }
    if (digitalRead(SW) == LOW) {
      Serial.print("homed");
      distanceTracker = 0;
      positionStacking = 0x00;
    }
  }
  else if (positionStacking == PICKPOSI){
    Serial.println("pickposition");
     Serial.println(LAGORI5POS - distanceTracker);
    moveDistance(PICKPOS - distanceTracker);
    distanceTracker += PICKPOS - distanceTracker;
    positionStacking = 0x00;
  }
  Serial.println();
}

//------------------------------------- FUNCTION DEFINITIONS ------------------------------------------

//------------------------------------- LAGORI FUNCTIONS ------------------------------
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
  //  Serial.print(dist);
  //  Serial.print("\t");
  if (dist < 0) {
    dist = -dist;
    long int steps = (STEPS_PER_REV * dist) / DISTANCE_PER_REV;
    //    Serial.println(steps);
    for (int i = 1; i <= steps; i++) {
      moveDown(10);
    }
  }
  else {
    long int steps = (STEPS_PER_REV * dist) / DISTANCE_PER_REV;
    //    Serial.println(steps);
    for (int i = 1; i <= steps; i++) {
      moveUp(10);
    }
  }
}

void closeGripper() {
  Serial.println("Closing Gripper");
  digitalWrite(GRIPPER1, HIGH);
  digitalWrite(GRIPPER2, LOW);
}

void openGripper() {
  Serial.println("Opening Gripper");
  digitalWrite(GRIPPER1, LOW);
  digitalWrite(GRIPPER2, HIGH);
}

void closeMechanism() {
  digitalWrite(INOUT, HIGH);
}

void openMechanism() {
  digitalWrite(INOUT, LOW);
}

//------------------------------------- BALL PICKING FUNCTIONS ---------------------------------------

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
