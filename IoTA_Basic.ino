#include "Arduino.h"
#include "IoTA_Basic.h"

IoTA_Basic iota(8); 

void setup() {  
  iota.begin(); 

  //set plan check point 
  iota.setCP(0,0);
  iota.setCP(1,4);

  iota.setSpeed(130); 

  planning(); 
}

void loop() {  
  iota.homeScreen();
}

void planning(){  
  iota.setPlan(0, BLACK_LINE, IGNORE_SENSOR, FORWARD, 800, 255, PID_NORMAL);
  iota.setPlan(1, BLACK_LINE, RIGHT_DETECT, TURN_RIGHT, 1100, 255, PID_NORMAL);
  iota.setPlan(2, BLACK_LINE, RIGHT_DETECT, TURN_RIGHT, 800, 255, PID_NORMAL); 
  iota.setPlan(4, BLACK_LINE, CENTER_RIGHT_DETECT, TURN_RIGHT, 100, 200, PID_LOW); 
  iota.setPlan(5, BLACK_LINE, CENTER_RIGHT_DETECT, TURN_RIGHT, 2000, 255, PID_LOW); 
  iota.setPlan(6, BLACK_LINE, CENTER_RIGHT_DETECT, FORWARD, 0, 0, PID_NORMAL); 

  iota.setFinish(6, AFTER_ACTION);
}
 
