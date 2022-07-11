#include<stdio.h>
#include<stdlib.h>
#include<string.h>

#define MAX 30
#define TEMPORARY 1
#define PERMANENT 2
#define NIL -1
#define INFINITY 99999

#include<stdio.h>
#include<stdlib.h>
#include<string.h>

#define MAX 30
#define TEMPORARY 1
#define PERMANENT 2
#define NIL -1
#define INFINITY 99999
/////////////////////////////////////////////////////////
#include <RoboClaw.h>
#include <Wire.h>
#include <MPU6050_light.h>

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LASER -------------------------------------------------
//---------------------------------------------------------------------------------------------------------
#define xlaser A1
#define xanalog A2
#define ylaser A3
#define yanalog A4

int x_pid(int xtarget, int xcurrent);
int y_pid(int ytarget, int ycurrent);

int xfilter(int xfinal);
int yfilter(int yfinal);

#define xKP 7 
#define xKI 0
#define xKD 7
int prevxerror=0;

#define yKP 7
#define yKI 0
#define yKD 7
int prevyerror=0;

int xprev =0;
int prevtx = 0;
int curtx;
float xfinal;
float dlpfx;

int yprev =0;
int prevty = 0;
int curty;
float yfinal;
float dlpfy;

int xsum = 0;
int ysum = 0;
//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- GYRO --------------------------------------------------
//---------------------------------------------------------------------------------------------------------
MPU6050 mpu(Wire);
bool gyro_setup();
int16_t gyro_read();
int32_t gyro_pid(int16_t target, int16_t current);
int comp = 0;
#define GYROSPEED 50
int targetAngle = 0;
uint8_t KP_Gyro = 15;
uint8_t KI_Gyro = 0;
uint8_t KD_Gyro = 0;
//Error variables
long int current_gyro_error = 0, prev_gyro_error = 0, p_gyro_error = 0, i_gyro_error = 0, d_gyro_error = 0;
float P_gyro = 0, I_gyro = 0, D_gyro = 0;
//Others
long int final_w = 0;

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
/////////////////////////////////////////////////////////
struct Vertex
{
  char name[50];
  int status;
  int predecessor;
  int pathLength;
  float xcord;
  float ycord;
};

struct Vertex vertexList[MAX];

int adj[MAX][MAX];
int n = 0;
int e = 0;


int getIndex(char s[]);
void insertVertex(char s[]);
void insertEdge(char s1[], char s2[], int wt);

void findPaths(char s[]);
void findPath(int s, int v);
void dijkstra(int s);
int tempVertexMinPL();

int counterpromax = 0;


void setup()
{
  insertVertex("Zero");
  insertVertex("One");
  insertVertex("Two");
  insertVertex("Three");

  insertEdge("Zero", "One", 11);
  insertEdge("One", "Two", 12);
  insertEdge("Two", "Three", 13);

  insertEdge("One", "Zero", 11);
  insertEdge("Two", "One", 12);
  insertEdge("Three", "Two", 13);


  Serial.begin(115200);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  roboclaw_setup();
  gyro_setup();
  /////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  /*
    0------------------------------------1
                                         \
                                          \
                                           \
                                            \
                                             2
                                             |
                                             |
                                             |
                                             |
                                             |
                                             |
                                             |
                                             |
                                             |
                                             |
                                             3
  */
  vertexList[0].xcord = 4050;
  vertexList[0].ycord = 0.0;
  vertexList[1].xcord = 1500;
  vertexList[1].ycord = 0.0;
  vertexList[2].xcord = 750;
  vertexList[2].ycord = 1500;
  vertexList[3].xcord = 750;
  vertexList[3].ycord = 3600;

  findPaths("Zero");//initial position to ball rack---------------------- assign this to one button
  findPaths("Three");//ball rack to initial position---------------------- assign this to another
}



void findPaths(char source[])
{
  counterpromax += 1;
  int s, v;
  s = getIndex(source);

  dijkstra(s);

  if (counterpromax == 1)
  {
    findPath(s, 3);
  }
  else if (counterpromax == 2)
  {
    findPath(s, 0);
  }
}

void dijkstra(int s)
{
  int v, c;

  for (v = 0; v < n; v++)
  {
    vertexList[v].status = TEMPORARY;
    vertexList[v].pathLength = INFINITY;
    vertexList[v].predecessor = NIL;
  }

  vertexList[s].pathLength = 0;

  while (1)
  {
    c = tempVertexMinPL();

    if (c == NIL)
      return;

    vertexList[c].status = PERMANENT;

    for (v = 0; v < n; v++)
    {
      if ( adj[c][v] != 0 && vertexList[v].status == TEMPORARY )
        if ( vertexList[c].pathLength + adj[c][v] < vertexList[v].pathLength)
        {
          vertexList[v].predecessor = c;
          vertexList[v].pathLength = vertexList[c].pathLength + adj[c][v];
        }
    }
  }
}

int tempVertexMinPL()
{
  int min = INFINITY;
  int v, x = NIL;

  for (v = 0; v < n; v++)
  {
    if (vertexList[v].status == TEMPORARY && vertexList[v].pathLength < min)
    {
      min = vertexList[v].pathLength;
      x = v;
    }
  }
  return x;
}
  int i, u;
  int path[MAX];
  int sd = 0;
  int count = 0;
void findPath(int s, int v)
{

  while (v != s)
  {
    count++;
    path[count] = v;
    u = vertexList[v].predecessor;
    sd += adj[u][v];
    v = u;
  }
  count++;
  path[count] = s;
  //  for (i = count; i >= 1; i--)
  //  {
  //    if (i != 1)
  //    {
  //      x_pid(path[i - 1]].xcord - vertexList[path[i]].xcord,xsum);
  //      y_pid(vertexList[path[i - 1]].ycord - vertexList[path[i]].ycord,ysum);
  //    }
  //  }
}


int getIndex(char s[])
{
  int i;
  for (i = 0; i < n; i++)
    if ( strcmp(s, vertexList[i].name) == 0 )
      return i;
}

void insertVertex(char s[])
{
  strcpy(vertexList[n].name, s);
  n++;
}
void insertEdge(char s1[], char s2[], int wt)
{
  int u = getIndex(s1);
  int v = getIndex(s2);
  adj[u][v] = wt;
  e++;
}

void loop()
{
  for (int i = count; i >= 1; i--)
  {
    if (i != 1)
    {


      int xsum = xfilter(analogRead(xlaser));
      int ysum = yfilter(analogRead(ylaser));

      int xway = x_pid(vertexList[path[i - 1]].xcord - vertexList[path[i]].xcord, xsum);
      int yway = y_pid(vertexList[path[i - 1]].ycord - vertexList[path[i]].ycord, ysum);


      drive(xway, yway, 0);
    }
  }

}

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
  if (current_gyro_error > 20) {
    KP_Gyro = 6;
    KI_Gyro = 1;
    KD_Gyro = 8;
  }
  else {
    KP_Gyro = 7;
    KI_Gyro = 0;
    KD_Gyro = 1;
  }
  p_gyro_error = current_gyro_error;
  i_gyro_error = prev_gyro_error + current_gyro_error;
  d_gyro_error = current_gyro_error - prev_gyro_error;

  P_gyro = KP_Gyro * p_gyro_error;
  I_gyro = KI_Gyro * i_gyro_error;
  D_gyro = KD_Gyro * d_gyro_error;
  int corr = -(P_gyro + I_gyro + D_gyro);
  prev_gyro_error = current_gyro_error;
  if (current_gyro_error > 20) {
    if (corr > GYROSPEED) {
      corr = GYROSPEED;
    }
    else if (corr < -GYROSPEED) {
      corr = -GYROSPEED;
    }
  }
  return corr;
}
//---------------------------------------------------------------------------------------------------------
//-------------------------------------------------- ROBOCLAW ---------------------------------------------
//---------------------------------------------------------------------------------------------------------
bool roboclaw_setup() {
  roboclaw.begin(115200);
}
//---------------------------------------------------------------------------------------------------------- -
//-------------------------------------------------- MOVEMENT ---------------------------------------------
//---------------------------------------------------------------------------------------------------------- -
bool drive(int x, int y, int w) {
  M1Speed = 0.33 * w + 0.58 * y - 0.33 * x;
  M2Speed = 0.33 * w - 0.58 * y - 0.33 * x;
  M3Speed = -0.33 * w - 0.67 * x;

  M1Speed = map(M1Speed, -127, 127, 10, 117);
  M2Speed = map(M2Speed, -127, 127, 10, 117);
  M3Speed = map(M3Speed, -127, 127, 10, 117);
  roboclaw.ForwardBackwardM1(address1, -M1Speed);
  roboclaw.ForwardBackwardM2(address2, -M2Speed);
  roboclaw.ForwardBackwardM2(address1, -M3Speed);
  return true;
}

//---------------------------------------------------------------------------------------------------------
//------------------------------------------------- LASER -------------------------------------------------
//---------------------------------------------------------------------------------------------------------
int x_pid(int xtarget , int xcurrent)
{
  int error = xtarget - xcurrent;
  int prop_error = error;
  int integral_error = error + prevxerror;
  int diff_error = error - prevxerror;
  prevxerror = error;
  return (xKP * prop_error) + (xKI * integral_error) + (xKD * diff_error);
}

int y_pid(int ytarget , int ycurrent)
{
  int error = ytarget - ycurrent;
  int prop_error = error;
  int integral_error = error + prevyerror;
  int diff_error = error - prevyerror;

  prevyerror = error;
  return (yKP * prop_error) + (yKI * integral_error) + (yKD * diff_error);
}

int xfilter(int xfinal)
{
  curtx = millis();
  //xfinal = analogRead(xlaser);
  dlpfx = (xfinal - xprev) / (curtx - prevtx);
  prevtx = millis();
  if (dlpfx > 0.005)
  {
    xfinal = xprev;
  }
  xprev = xfinal;
  return xfinal;
}

int yfilter(int yfinal)
{
  curty = millis();
  //yfinal = analogRead(ylaser);
  dlpfy = (yfinal - yprev) / (curty - prevty);
  prevty = millis();
  if (dlpfy > 0.005)
  {
    yfinal = yprev;
  }
  yprev = yfinal;
  return yfinal;
}
