
int LX, LY, RX, RY;
int S, T, C, X, U, D, L, R, R1, R2, R3, L1, L2, L3, START, SELECT;

#define MOSFET1 4  //Connected to STM 10B from bottom //2&4 DOWN/UP
#define MOSFET2 5  //Connected to STM 10A from bottom //3&1 DOWN/UP
#define MOSFET3 6  //Connected to STM 11B from bottom // BACK
#define MOSFET4 7   //Connected to STM 11A from bottom // FRONT

void setup() {
  Serial.begin(115200);  // put your setup code here, to run once:
  Serial1.begin(9600);
  pinMode(MOSFET1, OUTPUT);
  pinMode(MOSFET2, OUTPUT);
  pinMode(MOSFET3, OUTPUT);
  pinMode(MOSFET4, OUTPUT);

}

void receiveData() {
  //Serial1.begin(9600);
//  if (Serial1.available()) {
    //Serial.println("Called");
    int data = Serial1.read();
    //Serial.print("Data is\t");
    //Serial.println(data);
    if (data == 201) {
      while (!Serial1.available());
      LX = Serial1.read();
      Serial.print(LX);
      Serial.print("\t");
    }
    else if (data == 202) {
      while (!Serial1.available());
      LY = Serial1.read();
      Serial.print(LY);
      Serial.print("\t");
    }
    else if (data == 203) {
      while (!Serial1.available());
      RX = Serial1.read();
      Serial.print(RX);
      Serial.print("\t");
    }
    else if (data == 204) {
      while (!Serial1.available());
      RY = Serial1.read();
      Serial.println(RY);
    }
    else {
      switch (data) {
        case 237:
          T = 1;
          Serial.println("Triangle is Pressed");
          break;
        case 238:
          S = 1;
          Serial.println("Square is Pressed");
          break;
        case 239:
          X = 1;
          Serial.println("X is Pressed");
          break;
        case 240:
          C = 1;
          Serial.println("Circle is Pressed");
          break;
        case 241:
          U = 1;
          Serial.println("Up is Pressed");
          break;

        case 242:
          L = 1;
          Serial.println("Left is Pressed");
          break;

        case 243:
          D = 1;
          Serial.println("Down is Pressed");
          break;

        case 244:
          R = 1;
          Serial.println("Right is Pressed");
          break;

        case 231:
          L1 = 1;
          Serial.println("L1 is Pressed");
          break;

        case 232:
          R1 = 1;
          Serial.println("R1 is Pressed");
          break;

        case 233:
          L2 = 1;
          Serial.println("L2 is Pressed");
          break;

        case 234:
          R2 = 1;
          Serial.println("R2 is Pressed");
          break;

        case 235:
          L3 = 1;
          Serial.println("L3 is Pressed");
          break;

        case 236:
          R3 = 1;
          Serial.println("R3 is Pressed");
          break;

        case 245:
          START = 1;
          Serial.println("Start is Pressed");
          break;

        case 246:
          SELECT = 1;
          Serial.println("Select is Pressed");
          break;

      }
    }
  }
}

int ForwardFlag = 0;
int StopFlag = 1;
int BackFlag = 0;



void loop() {
  receiveData();
  if (U) {
    U = 0;
    digitalWrite(MOSFET3, HIGH);
    digitalWrite(MOSFET4, LOW);
    delay(200);
    digitalWrite(MOSFET1, LOW);
    digitalWrite(MOSFET2, LOW);
    delay(200);
    digitalWrite(MOSFET1, HIGH);
    digitalWrite(MOSFET2, HIGH);
  }
  else if (D) {
    D = 0;
    digitalWrite(MOSFET3, LOW);
    digitalWrite(MOSFET4, HIGH);
    delay(200);
    digitalWrite(MOSFET1, LOW);
    digitalWrite(MOSFET2, LOW);
    delay(200);
    digitalWrite(MOSFET1, HIGH);
    digitalWrite(MOSFET2, HIGH);
  }
  else if (X) {
    X = 0;
  }

}
