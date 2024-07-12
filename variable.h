float kp = 30; 
float ki = 0.0;
float kd = 50;
float ts = 10;
float minSpd = -255;
float maxSpd = 255;
// int8_t Err = 0, ErrD = 0, lastErr = 0;
long ErrI = 0;

int spd = 150;

bool dLost = 0;
bool dRight = 0;
bool dLeft = 0;
bool dFinish = 0;
int8_t statusLine = 0;
bool detectSens = 0;  
int n = 0;

int m = 0, m1 = 0;
int maxm = 3;
int minm = 0;
int maxm1 = 3;
int minm1 = 0;

// int error = 0;

unsigned long timer = 0;
 
bool initWifi = 0;

uint8_t enableScan = 1;
 