#define GRIPPER1 A2
#define GRIPPER2 A3
#define INOUT A4
#define DIR 10
#define STEP 11

#define BAUD 115200
#define BAUDRATE ((F_CPU/(BAUD*16UL)-1))

#define STEPS_PER_REV 6400
#define DISTANCE_PER_REV 42 //mm

volatile char temp = 'j';



void setup() {
  pinMode(13, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  UBRR0H = BAUDRATE >> 8;
  UBRR0L = BAUDRATE;
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);                                // Use 8-bit character sizes
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);                  // Turn on the transmission, reception, and Receive interrupt
  sei();
  pinMode(GRIPPER1, OUTPUT);
  pinMode(GRIPPER2, OUTPUT);
  pinMode(INOUT, OUTPUT);
}

void loop() {
  //  while(temp == 'j');
  if (temp == '0') {
    digitalWrite(13, HIGH);
    moveUp(5);
  }

  if (temp == '5') {
    digitalWrite(13, LOW);
    moveDown(5);
  }


  if(temp == '7'){
    openGripper();
  }
  else if(temp == '6'){
    closeGripper();
  }
  else if(temp == '8'){
    openMechanism();
  }
  else if(temp == '9'){
    closeMechanism();
  }
}


void closeGripper() {
//  Serial.println("Closing Gripper");
  digitalWrite(GRIPPER1, HIGH);
  digitalWrite(GRIPPER2, LOW);
}

void openGripper() {
//  Serial.println("Opening Gripper");
  digitalWrite(GRIPPER1, LOW);
  digitalWrite(GRIPPER2, HIGH);
}

void closeMechanism() {
  digitalWrite(INOUT, HIGH);
}

void openMechanism() {
  digitalWrite(INOUT, LOW);
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

ISR(USART_RX_vect)
{
  temp = UDR0;
}
