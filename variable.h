struct setConfig {
  int8_t CP[10];
  uint8_t refADC[maxSensor];

}; setConfig EEc;

uint8_t addRef[maxSensor + 1] = {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t addMaze = 10; 

uint8_t s[maxSensor + 1];

char buf[64];

uint8_t refADC[maxSensor + 1];

uint8_t maxADC[maxSensor + 1];
uint8_t minADC[maxSensor + 1];

uint8_t maxMaze = 0;
char maze[200];

#define maxMenu 4
String menuSet[maxMenu + 1] = {
  "START ROBOT",
  "CHECK POINT:", 
  "CALIBRATE SENSORS",
  "TEST MOTOR:",
};

float kp = 30; 
float ki = 0.0;
float kd = 50;
float ts = 10;
float minSpd = -255;
float maxSpd = 255;
int8_t Err = 0, ErrD = 0, lastErr = 0;
long ErrI = 0;

int spd = 150;

bool dLost = 0;
bool dRight = 0;
bool dLeft = 0;
bool dFinish = 0;
int8_t statusLine = 0;
bool detectSens = 0;

const int8_t CENTER = 0;
const int8_t LEFT = 1;
const int8_t RIGHT = 2;
const int8_t CEN_LEFT = 3;
const int8_t CEN_RIGHT = 4; 
const int8_t LEFT_RIGHT = 5; 
const int8_t LOSS = 6;
const int8_t FINISH = 7;

int n = 0;

int m = 0, m1 = 0;
int maxm = 3;
int minm = 0;
int maxm1 = 3;
int minm1 = 0;

int error = 0;

unsigned long timer = 0;


bool initWifi = 0;

uint8_t enableScan = 1;

String statusSens[8] = {
  "CENTER",
  "LEFT",
  "RIGHT",
  "CEN_LEFT",
  "CEN_RIGHT",
  "LEFT_RIGHT",
  "LOSS",
  "FINISH"
};