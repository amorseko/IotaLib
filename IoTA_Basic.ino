#include "IoTA_Basic.h"

IoTA_Basic iota; 

void setup() { 
  // iota.setWiFi("Kkrbt_mobile", "Kkrbt2024");
  iota.setPinSensor(12, 14, 33, 32, 35, 34, 39, 36);
  // iota.setPinSensor(26, 25, 33, 32, 35, 34, 39, 36 );
  iota.setPinTombol(16, 15, 19, 23);
  iota.begin(); 
}

void loop() {  
  iota.homeScreen();
}

void planning(){

}
