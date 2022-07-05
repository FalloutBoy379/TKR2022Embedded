//###################################################################################################################################################################
//############################################################ INCLUDES AND CLASS DEFINITIONS #######################################################################
//###################################################################################################################################################################

#include <RoboClaw.h>
#include <MPU6050_light.h>
#include "Wire.h"
#include <AutoPID.h>

class BTS {
  public:
    BTS(int pwm, int dir1, int dir2) {
      pinMode(pwm, OUTPUT);
      pinMode(dir1, OUTPUT);
      pinMode(dir2, OUTPUT);
      PWM = pwm;
      DIR1 = dir1;
      DIR2 = dir2;
      setMaxPWM(255);
    }
    void setMaxPWM(int maxVel) {
      MAXVEL = maxVel;
    }
    void runMotor(int vel) {
      if (vel > MAXVEL) {
        vel = MAXVEL;
      }
      else if (vel < -MAXVEL) {
        vel = -MAXVEL;
      }

      if (vel >= 0) {
        runForward(vel);
      }
      else if (vel < 0) {
        runBackward(-vel);
      }
    }
  private:
    int DIR1, DIR2, PWM, MAXVEL = 255;

    void runForward(int vel) {
      Serial.print("Running Ahead with speed: ");
      Serial.println(vel);
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, LOW);
      analogWrite(PWM, vel);
    }

    void runBackward(int vel) {
      Serial.print("Running Backward with speed: ");
      Serial.println(vel);
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, HIGH);
      analogWrite(PWM, vel);
    }
};
class Hopper {
  public:
    void Init() {
      pinMode(CENTERPISTONOUT, OUTPUT);
      pinMode(CENTERPISTONIN, OUTPUT);
      pinMode(LEFTPISTON, OUTPUT);
      pinMode(RIGHTPISTON, OUTPUT);
      centerPiston(IN);
      leftPiston(OUT);
      rightPiston(OUT);
    }

    void exec() {
      if (((rState != defaultRight) || (lState != defaultLeft)) && sideFlag == 0) {
        sideFlag = 1;
        currentTimeSide = millis();
      }

      if ((cState != defaultCenter) && centerFlag == 0) {
        centerFlag = 1;
        currentTimeCenter = millis();
      }
      if (millis() - currentTimeSide > 3000) {
        sideFlag = 0;
        lState = defaultLeft;
        rState = defaultRight;
      }

      if (millis() - currentTimeCenter > 300) {
        cState = defaultCenter;
        centerFlag = 0;
      }
      centerPiston(cState);
      leftPiston(lState);
      rightPiston(rState);
    }

    bool cState = defaultCenter;
    bool lState = defaultLeft;
    bool rState = defaultRight;

  private:
    int sideFlag = 1;
    int centerFlag = 1;
    long long int currentTimeSide, currentTimeCenter;
    const bool defaultCenter = IN;
    const bool defaultLeft = OUT;
    const bool defaultRight = OUT;


    void centerPiston(bool state) {
      digitalWrite(CENTERPISTONIN, !state);
      digitalWrite(CENTERPISTONOUT, state);
    }

    void leftPiston(bool state) {
      digitalWrite(LEFTPISTON, state);
    }

    void rightPiston(bool state) {
      digitalWrite(RIGHTPISTON, state);
    }
};

//###################################################################################################################################################################
//##################################################################################### GYRO ########################################################################
//###################################################################################################################################################################
MPU6050 mpu(Wire);
double yawGyro;;
double correction;
double target;
double Kp_Gyro = 6000, Ki_Gyro = 500, Kd_Gyro = 0;
AutoPID drivePID(&yawGyro, &target, &correction, -25000, 25000, Kp_Gyro, Ki_Gyro, Kd_Gyro);
bool gyro_setup();
int16_t gyro_read();

//###################################################################################################################################################################
//################################################################################# ROBOCLAW ########################################################################
//###################################################################################################################################################################
RoboClaw roboclaw(&Serial1, 10000);
#define address1 128
#define address2 129



//###################################################################################################################################################################
//##################################################################################### RGB #########################################################################
//###################################################################################################################################################################
#define REDLED 9
#define GREENLED 11
#define BLUELED 10
void lights_setup();
void lights_write(int r, int g, int b);


//###################################################################################################################################################################
//##################################################################################### PS ##########################################################################
//###################################################################################################################################################################

#define BAUD 9600
#define BAUDRATE ((F_CPU / (BAUD * 16UL) - 1))
#define pwm_range 200
#define PS_L1 0
#define PS_R1 1
#define PS_L2 19
#define PS_R2 20
#define PS_L3 16
#define PS_R3 17
#define PS_TRIANGLE 6
#define PS_SQUARE 7
#define PS_CROSS 8
#define PS_CIRCLE 9
#define PS_UP 10
#define PS_LEFT 11
#define PS_DOWN 12
#define PS_RIGHT 13
#define PS_OPTIONS 14
#define PS_SHARE 15
#define PS_MIC 2
#define PS_BUTTON 3

int prevMillis = 0;
int xj1Offset = 0, yj1Offset = 0, xj2Offset = 0, yj2Offset = 0, calibrateFlag = 0, prevComp = 0;

int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0;  //analog values(serially received from remote);
int butt[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };               //digital values(serially received from remote);

uint8_t RX[16] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0;
uint8_t TX[16] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
int flag[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;


bool ps_setup();
void ps_read();


//###################################################################################################################################################################
//##################################################################################### THROWING ####################################################################
//###################################################################################################################################################################
#define POTPIN A1
#define UPPERPWM 46
#define UPPERM1 47
#define UPPERM2 48
#define LOWERPWM 44
#define LOWERM1 40
#define LOWERM2 42

double Kp = 300, Ki = 150, Kd = 100;
#define OUTPUT_MAX 15000
#define OUTPUT_MIN -15000
double PotSetPoint, outputVal;
const int analogInPin = A1;  // Analog input pin that the potentiometer is attached to
double sensorValue = 0;
AutoPID potPID(&sensorValue, &PotSetPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, Kp, Ki, Kd);

int throwingState = 0;
BTS upperMotor(UPPERPWM, UPPERM1, UPPERM2), lowerMotor(LOWERPWM, LOWERM1, LOWERM2);
void throwBall(BTS& motor1, int upperVel, BTS& motor2, int lowerVel);


//###################################################################################################################################################################
//##################################################################################### HOPPER ######################################################################
//###################################################################################################################################################################
#define CENTERPISTONOUT 34
#define CENTERPISTONIN 32
#define LEFTPISTON 28
#define RIGHTPISTON 30

#define IN 1
#define OUT 0
Hopper hopper;






int statusPS = 0;
//#####################################################################################################################################################################################################################################################################
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  target = 0;
  PotSetPoint = 700;
  lights_write(100, 120, 140);
  Serial.println("Starting");
  gyro_setup();
  Serial.println("Gyro setup done");
  
  ps_setup();
  Serial.println("Ps setup done");
  roboclaw.begin(115200);
  statusPS = 0;
  hopper.Init();
  potPID.setBangBang(100);
  potPID.setTimeStep(1);
  drivePID.setBangBang(180);
  drivePID.setTimeStep(1);

}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {

  if (RX_count == 1) {
    yawGyro = gyro_read();
    
    sensorValue = analogRead(analogInPin);

    ps_read();
    if(abs(target - yawGyro) < 10){
      Ki_Gyro = 32000;
      Kp_Gyro = 6000;
      Kd_Gyro = 0;
    }
    else{
      Kp_Gyro = 12000;
      Ki_Gyro = 500;
      Kd_Gyro = 20000;
    }
    drivePID.run();
    potPID.run();
    statusPS = 1;
    lights_write(0, 255, 0);
    //    outputVal = map(xj2, -200, 200, -30000, 30000);
    //    if (Output < 4000 && Output > -4000) {
    //      Output = 0;
    //    }
    //    int output = output
    Serial.print(Kp_Gyro);
    Serial.print("\t");
    Serial.print(Ki_Gyro);
    Serial.print("\t");
    Serial.print(Kd_Gyro);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print("output = ");
    Serial.print(correction);
    Serial.print("\t");
    Serial.print("sensor = ");
    Serial.println(yawGyro);
    roboclaw.DutyM1(address1, -outputVal);
    drive(0, 0, correction);
    hopper.exec();
  } else {
    statusPS = 0;
  }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#####################################################################################################################################################################################################################################################################



void lights_setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  pinMode(GREENLED, OUTPUT);

  digitalWrite(REDLED, HIGH);
  digitalWrite(GREENLED, LOW);
  digitalWrite(BLUELED, LOW);
}

void lights_write(int r, int g, int b) {
  analogWrite(REDLED, r);
  analogWrite(GREENLED, g);
  analogWrite(BLUELED, b);
}

bool ps_setup() {
  UBRR3H = BAUDRATE >> 8;
  UBRR3L = BAUDRATE;
  UCSR3B = 0b10011000;  //enable RXEN TXEN
  UCSR3C = 0b00000110;
}

ISR(USART3_RX_vect) {
  RX_count = 1;
  RX_raw = UDR3;
  if ((RX_raw > 200) && (RX_raw < 255)) {
    RX_ad = RX_raw;
    if ((RX_raw > 230) && (RX_raw < 252)) {
      uint8_t r_temp0 = (RX_raw - 231);
      butt[r_temp0] = 1;
    }
  } else if ((RX_raw >= 0) && (RX_raw < 201)) {
    uint8_t r_temp1 = (RX_ad - 201);
    if (r_temp1 < 16) {
      RX[r_temp1] = RX_raw;
    }
  }
}

void ps_read() {
  RX_count = 0;

  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, -pwm_range, (pwm_range));
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));
  pot1 = map(RX[4], 0, RX_range, 0, (pwm_range));  //R2
  pot2 = map(RX[5], 0, RX_range, 0, (pwm_range));  //L2

  if (butt[PS_SHARE]) {
    PotSetPoint = 700;
    target = 0;
    Serial.println("Share");
    butt[PS_SHARE] = 0;
  }
  if (butt[PS_OPTIONS] == 1) {
    PotSetPoint = 700;
    target = 90;
    Serial.println("Options");
    butt[PS_OPTIONS] = 0;
    ;
  }
  if (butt[PS_UP] == 1) {
    PotSetPoint = 780;
    target = -30;
    Serial.println("Up");
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1) {
    PotSetPoint = 780;
    target = 30;
    Serial.println("Down");
    butt[PS_DOWN] = 0;
  }
  if (butt[PS_LEFT] == 1) {
    PotSetPoint = 740;
    target = 75;
    Serial.println("left");
    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1) {
    PotSetPoint = 740;
    target = -75;
    Serial.println("right");
    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1) {
    hopper.lState = IN;
    Serial.println("square");
    butt[PS_SQUARE] = 0;
  }
  if (butt[PS_CIRCLE] == 1) {
    hopper.rState = IN;
    Serial.println("circle");
    butt[PS_CIRCLE] = 0;
  }
  if (butt[PS_TRIANGLE] == 1) {
    hopper.cState = OUT;
    Serial.println("triangle");
    butt[PS_TRIANGLE] = 0;
  }
  if (butt[PS_CROSS] == 1) {
    if (throwingState == 0) {
      throwBall(upperMotor, -100 , lowerMotor, 100);
    }
    else {
      throwBall(upperMotor, 0, lowerMotor, 0);
    }
    Serial.println("cross");
    butt[PS_CROSS] = 0;
  }
  if (butt[PS_L1] == 1) {
    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1) {
    Serial.println("R1");
    butt[PS_R1] = 0;
  }
  if (pot2 >= 180)  //L2
  {
    Serial.println("L2");
  }
  if (pot1 >= 180)  //R2
  {
    Serial.println("R2");
  }
  if (butt[PS_L3] == 1) {
    Serial.println("L3");
    butt[PS_L3] = 0;
  }
  if (butt[PS_R3] == 1) {
    Serial.println("R3");
    butt[PS_R3] = 0;
  }
  if (butt[PS_MIC] == 1) {
    Serial.println("Mic");
    butt[PS_MIC] = 0;
  }
  if (butt[PS_BUTTON] == 1) {
    Serial.println("PS Button");
    butt[PS_BUTTON] = 0;
  }
}


void drive(double A_x, double A_y, double ang_s)
{
  double F1, F2 ,F3;

  F1 = (0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F2 = -(0.5773 * A_x) + (0.333 * A_y) + (0.333 * ang_s);
  F3 = (0*A_x) - (0.666*A_y) + (0.333*ang_s);

  if (F1 > 25000)
  {
    F1 = 25000;
  }
  if (F2 > 25000)
  {
    F2 = 25000;
  }
  if (F1 < -25000)
  {
    F1 = -25000;
  }
  if (F2 < -25000)
  {
    F2 = -25000;
  }
  
  if (F3 < -25000)
  {
    F3 = -25000;
  }
  if (F3 < -25000)
  {
    F3 = -25000;
  }

  roboclaw.DutyM2(128, -F1);
  roboclaw.DutyM2(129, F2);
  roboclaw.DutyM1(129, -F3);
}



bool gyro_setup() {
  Wire.begin();
  mpu.setAddress(0x69);
  byte status = mpu.begin();
  while (status != 0) {
    Serial.println("FATALITY      -       Could not connect!!");
  }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();
  Serial.println("Done!\n");
  delay(1000);
  return true;
}

int16_t gyro_read() {
  mpu.update();
  return mpu.getAngleZ();
}

//###################################################################################################################################################################
//##################################################################################### THROWING ####################################################################
//###################################################################################################################################################################

void throwBall(BTS& motor1, int upperVel, BTS& motor2, int lowerVel) {
  if (upperVel != 0  ||  lowerVel != 0) {
    throwingState = 1;
  }
  else {
    throwingState = 0;
  }
  motor1.runMotor(upperVel);
  motor2.runMotor(lowerVel);
}
