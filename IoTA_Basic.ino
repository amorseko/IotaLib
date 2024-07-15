#include "Arduino.h"
#include "IoTA_Basic.h"

IoTA_Basic iota(SENSOR_8); 
#define NO_ACTION   0, 0, 0 
#define TURN_RIGHT  200, -160, 150 
#define TURN_LEFT   -160, 200, 150
#define FORWARD     100, 100, 100
#define BACKWARD    100, 100, 150
#define STOP        0, 0, 200 

void setup() {  
  iota.begin();

  iota.setSpeed(120); 

  iota.setCP(M1,0,1,2,0,0,0,0,0,0,0); 

  planning1(); 
  planning2(); 
  planning3(); 
  planning4(); 
}

void loop() {  
  iota.homeScreen();
}

void planning1(){   
  iota.setMemory(M1); 
  
  iota.setPlan(0, BLACK_LINE, ACTION_NO_SENSOR, FORWARD, 760, 255, PID_NORMAL);
  iota.setPlan(1, BLACK_LINE, RIGHT_DETECT, TURN_RIGHT, 1100, 255, PID_NORMAL);
  iota.setPlan(2, BLACK_LINE, RIGHT_DETECT, TURN_RIGHT, 700, 255, PID_NORMAL); 
  iota.setPlan(4, BLACK_LINE, LEFT_RIGHT_DETECT, 200, 10, 250, 400, 220, PID_HIGH); 
  iota.setPlan(5, BLACK_LINE, CENTER_RIGHT_DETECT, TURN_RIGHT, 1500, 255, PID_HIGH); 
  iota.setPlan(6, BLACK_LINE, LEFT_DETECT, 150, 180, 300, 1000, 255, PID_HIGH); 
  iota.setPlan(7, BLACK_LINE, RIGHT_DETECT, TURN_RIGHT, 400, 255, PID_HIGH); 
  iota.setPlan(8, BLACK_LINE, CENTER_RIGHT_DETECT, FORWARD, 0, 0, PID_HIGH); 

  iota.setFinish(8, AFTER_ACTION);
}

void planning2(){   
  iota.setMemory(M2);
  iota.copyMemory(M1, M2);
  iota.mirrorMemory(M2);
} 

void planning3(){    
  // iota.copyMemory(M1, M3); 
  iota.setMemory(M3);
  iota.setPlan(0, BLACK_LINE, ACTION_NO_SENSOR, FORWARD, 600, 255, PID_NORMAL);
  iota.setPlan(1, BLACK_LINE, LEFT_RIGHT_DETECT, FORWARD, 0, 0, PID_NORMAL); 
  iota.setPlan(2, WHITE_LINE, ACTION_NO_SENSOR, NO_ACTION, 100, 255, PID_NORMAL); 
  iota.setPlan(3, WHITE_LINE, RIGHT_DETECT, TURN_RIGHT, 1000, 200, PID_HIGH); 
  iota.setPlan(4, WHITE_LINE, LEFT_DETECT, TURN_LEFT, 1200, 180, PID_HIGH); 
  iota.setPlan(5, WHITE_LINE, CENTER_RIGHT_DETECT, TURN_RIGHT, 150, 255, PID_NORMAL); 
  iota.setPlan(6, WHITE_LINE, LEFT_RIGHT_DETECT, FORWARD, 0, 0, PID_NORMAL); 
  iota.setPlan(7, BLACK_LINE, ACTION_NO_SENSOR, NO_ACTION, 250, 255, PID_NORMAL); 
  iota.setPlan(8, BLACK_LINE, RIGHT_DETECT, TURN_RIGHT, 500, 255, PID_NORMAL); 
  iota.setPlan(9, BLACK_LINE, LEFT_DETECT, TURN_LEFT, 200, 255, PID_NORMAL); 
  iota.setPlan(10, BLACK_LINE, ALL_SENSOR_NOT_DETECT, TURN_LEFT, 1500, 200, PID_LOW); 

  iota.setFinish(10, AFTER_TIMER);

}

void planning4(){    
  iota.copyMirrorMemory(M3, M4); 
}
 
