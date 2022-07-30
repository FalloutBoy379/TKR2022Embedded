#include <TFMPlus.h>
#include <RoboClaw.h>
#include <Wire.h>
#include <MPU6050_light.h>

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LIGHTS -------------------------------------------------
//---------------------------------------------------------------------------------------------------------
#define REDLED 13
#define GREENLED 11
#define BLUELED 12

void lights_setup();
void lights_write(int r, int g, int b);
//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LIDAR -------------------------------------------------
//---------------------------------------------------------------------------------------------------------
TFMPlus tfmP;

bool lidar_setup();
bool lidar_read();
int16_t PID_lidar(int16_t currentValue, int16_t targetValue);

// Initialize variables
int16_t tfDist = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip

int16_t targetDist = 32;
#define KP_lidar 4
#define KI_lidar 1
#define KD_lidar 4
int prev_error_lidar = 0;

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LASER -------------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool laser_setup();
int16_t laser_read();
int64_t laser_pid(int targetValue, int currentValue);

float KP_Laser1 = 0;
float KI_Laser1 = 0;
float KD_Laser1 = 0;

int16_t current_laser1_error = 0, prev_laser1_error = 0, p_laser1_error = 0, i_laser1_error = 0, d_laser1_error = 0;
float P_laser1, I_laser1, D_laser1;
int64_t laser_pid();



//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- ENCODER -----------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool encoder_setup();
bool encoder_read();

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- GYRO --------------------------------------------------
//---------------------------------------------------------------------------------------------------------
MPU6050 mpu(Wire);
bool gyro_setup();
int16_t gyro_read();
int32_t gyro_pid(int16_t target, int16_t current);
int comp = 0;

int targetAngle = 0;
uint8_t KP_Gyro = 7;
uint8_t KI_Gyro = 0;
uint8_t KD_Gyro = 0;
//Error variables
long int current_gyro_error = 0, prev_gyro_error = 0, p_gyro_error = 0, i_gyro_error = 0, d_gyro_error = 0;
float P_gyro = 0, I_gyro = 0, D_gyro = 0;
//Others
long int final_w = 0;

//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- PS ---------------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool ps_read();
bool ps_setup();

#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))
#define pwm_range 115
#define PS_L1 0
#define PS_R1 1
#define PS_L2 2
#define PS_R2 3
#define PS_L3 4
#define PS_R3 5
#define PS_TRIANGLE 6
#define PS_SQUARE 7
#define PS_CROSS 8
#define PS_CIRCLE 9
#define PS_UP 10
#define PS_LEFT 11
#define PS_DOWN 12
#define PS_RIGHT 13
#define PS_START 14
#define PS_SELECT 15

int prevMillis = 0;
int xj1Offset = 0, yj1Offset = 0, xj2Offset = 0, yj2Offset = 0, calibrateFlag = 0, prevComp = 0;

int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0; //analog values(serially received from remote);
int butt[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //digital values(serially received from remote);

uint8_t RX[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0;
uint8_t TX[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
int flag[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;

//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- ROBOCLAW ---------------------------------------------
//---------------------------------------------------------------------------------------------------------
RoboClaw roboclaw(&Serial1, 10000);
bool roboclaw_setup();
#define address1 128
#define address2 129

//---------------------------------------------------------------------------------------------------------- -
//-------------------------------------------------- MOVEMENT ---------------------------------------------
//---------------------------------------------------------------------------------------------------------- -
bool drive(int x, int y, int w);
float M1Speed, M2Speed, M3Speed;

//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- OTHER ------------------------------------------------
//---------------------------------------------------------------------------------------------------------- -
float limitVar(float var, float target);
int loopVar = 0;
//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- MAIN FUNCTIONS ---------------------------------------
//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);   // Intialize terminal serial port
  lidar_setup();
  lights_setup();
  roboclaw_setup();
  gyro_setup();
  ps_setup();

  //  laser_setup();
}

void loop() {
  if (loopVar == 0) {
    loop2();
    lights_write(0, 255, 0);
  }
  else {
    lights_write(0, 0, 255);
    loop1();
  }
}

void loop2() {
  ps_read();
  //  int laserVal = laser_read();
  int w = gyro_pid(targetAngle, gyro_read());
  w = w + comp*10;
  if (w > 90) {
    w = 127;
  }
  else if (w < -90) {
    w = -127;
  }
  Serial.print(gyro_read());
  drive(-yj1, -xj1, w);
}

void loop1() {
  ps_read();
  //  int laserVal = laser_read();
  int w = gyro_pid(targetAngle, gyro_read());
  w = w + comp*10;
  if (w > 127) {
    w = 127;
  }
  else if (w < -127) {
    w = -127;
  }
  int16_t rawDist = filterLidarData(tfDist, targetDist);
  int16_t xLidar = PID_lidar(rawDist, targetDist);
  if (xLidar > 127) {
    xLidar = 127;
  }
  else if (xLidar < -127) {
    xLidar = -127;
  }
  if (abs(rawDist) < abs(targetDist)) {
    xj1 = 0;
  }
  else {

  }
  Serial.print(gyro_read());
  xj1 = xj1 + xLidar;
  Serial.print("\t");
  Serial.print(rawDist);
  Serial.print("\t");
  Serial.print(xj1);

  drive(-yj1, -xj1, w);
}


//-------------------------------------------------- DEFINITIONS ------------------------------------------
bool gyro_setup() {
  Wire.begin();
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

int32_t gyro_pid(int target, int current) {
  current_gyro_error = target - current;

  p_gyro_error = current_gyro_error;
  i_gyro_error = prev_gyro_error + current_gyro_error;
  d_gyro_error = current_gyro_error - prev_gyro_error;

  P_gyro = KP_Gyro * p_gyro_error;
  I_gyro = KI_Gyro * i_gyro_error;
  D_gyro = KD_Gyro * d_gyro_error;

  prev_gyro_error = current_gyro_error;
  return -(P_gyro + I_gyro + D_gyro);
}

bool roboclaw_setup() {
  roboclaw.begin(115200);
}

bool ps_setup() {
  UBRR3H = BAUDRATE >> 8;
  UBRR3L = BAUDRATE;
  UCSR3B = 0b10011000; //enable RXEN TXEN
  UCSR3C = 0b00000110;
}

ISR(USART3_RX_vect)
{
  RX_count = 1;
  RX_raw = UDR3;
  if ((RX_raw > 200) && (RX_raw < 255))
  {
    RX_ad = RX_raw;
    if ((RX_raw > 230) && (RX_raw < 247))
    {
      uint8_t r_temp0 = (RX_raw - 231);
      butt[r_temp0] = 1;
    }
  }
  else if ((RX_raw >= 0) && (RX_raw < 201))
  {
    uint8_t r_temp1 = (RX_ad - 201);
    if (r_temp1 < 16)
    {
      RX[r_temp1] = RX_raw;
    }
  }
}

bool ps_read() {
  RX_count = 0;
  yj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  xj1 = map(RX[1], 0, RX_range, pwm_range, (-pwm_range));
  yj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  xj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));

  if (butt[PS_START] == 1)
  {
    Serial.println("Start");
    butt[PS_START] = 0;
  }
  if (butt[PS_SELECT] == 1)
  {
    Serial.println("Select");
    butt[PS_SELECT] = 0;
  }
  if (butt[PS_UP] == 1)
  {
    Serial.println("Up");
    butt[PS_UP] = 0;
  }
  if (butt[PS_DOWN] == 1)
  {
    Serial.println("Down");
    butt[PS_DOWN] = 0;
  }
  if (butt[PS_LEFT] == 1)
  {
    Serial.println("left");
    butt[PS_LEFT] = 0;
  }
  if (butt[PS_RIGHT] == 1)
  {
    Serial.println("right");
    butt[PS_RIGHT] = 0;
  }
  if (butt[PS_SQUARE] == 1)
  {
    comp--;
    Serial.println("square");
    butt[PS_SQUARE] = 0;
  }
  if (butt[PS_CIRCLE] == 1)
  {
    comp++;
    Serial.println("circle");
    butt[PS_CIRCLE] = 0;
  }
  if (butt[PS_TRIANGLE] == 1)
  {
    targetAngle = 0;
    Serial.println("triangle");
    butt[PS_TRIANGLE] = 0;
  }
  if (butt[PS_CROSS] == 1)
  {
    targetAngle = 90;
    Serial.println("cross");
    butt[PS_CROSS] = 0;
  }
  if (butt[PS_L1] == 1)
  {
    loopVar = 0;
    Serial.println("L1");
    butt[PS_L1] = 0;
  }
  if (butt[PS_R1] == 1)
  {
    loopVar = 1;
    Serial.println("R1");
    butt[PS_R1] = 0;
  }
  if (butt[PS_L2] == 1)
  {
    butt[PS_L2] = 0;
  }
  if (butt[PS_R2] == 1)
  {
    butt[PS_R2] = 0;
  }
  if (butt[PS_L3] == 1)
  {
    Serial.println("L3");
    butt[PS_L3] = 0;
  }
  if (butt[PS_R3] == 1)
  {
    Serial.println("R3");
    butt[PS_R3] = 0;
  }
}

bool lidar_setup() {
  Serial2.begin(115200);  // Initialize TFMPLus device serial port.
  tfmP.begin( &Serial2);   // Initialize device library objectect and...
}

float limitVar(float var, float target) {
  if (var >= target) {
    var = target;
  }
  else if (var <= -target) {
    var = -target;
  }
  return var;
}

int16_t filterLidarData(int16_t &filteredData, float weight) {
  int16_t raw = 0;
  if (tfmP.getData( raw, tfFlux, tfTemp)) {
    filteredData = (weight * raw) + ((1.0 - weight) * filteredData);
    return raw;
  }
  else {
    return filteredData;
  }
}

int16_t PID_lidar(int16_t currentValue, int16_t targetValue) {
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_error_lidar;
  int diff_error = error - prev_error_lidar;

  prev_error_lidar = error;
  return (KP_lidar * prop_error) + (KI_lidar * integral_error) + (KD_lidar * diff_error);
}

bool drive(int x, int y, int w) {
  M1Speed = 0.33 * w + 0.58 * y - 0.33 * x;
  M2Speed = 0.33 * w - 0.58 * y - 0.33 * x;
  M3Speed = -0.33 * w - 0.67 * x;
  M1Speed = limitVar(M1Speed, 30000);
  M2Speed = limitVar(M2Speed, 30000);
  M3Speed = limitVar(M3Speed, 30000);

  M1Speed = map(M1Speed, -127, 127, 0, 127);
  M2Speed = map(M2Speed, -127, 127, 0, 127);
  M3Speed = map(M3Speed, -127, 127, 0, 127);
  Serial.print("\t");
  Serial.print(M1Speed);
  Serial.print("\t");
  Serial.print(-M2Speed);
  Serial.print("\t");
  Serial.println(-M3Speed);
  roboclaw.ForwardBackwardM1(address1, M1Speed);
  roboclaw.ForwardBackwardM2(address1, -M2Speed);
  roboclaw.ForwardBackwardM2(address2, -M3Speed);
  return true;
}

bool laser_setup() {
  pinMode(A1, INPUT);
}

int16_t laser_read() {
  int data = analogRead(A1);
  //  data = map(data, 213, 1024, 208, 12000);
  Serial.print(data);
  Serial.print("\t");
  return data;
}

int64_t laser_pid(int targetValue, int currentValue) {
  int error = targetValue - currentValue;

  int prop_error = error;
  int integral_error = error + prev_laser1_error;
  int diff_error = error - prev_laser1_error;

  prev_laser1_error = error;
  return (KP_Laser1 * prop_error) + (KI_Laser1 * integral_error) + (KD_Laser1 * diff_error);
}

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
