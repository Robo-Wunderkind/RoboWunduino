#include "robowunderkind.h"

RoboWunderkind RW = RoboWunderkind();

void setup() 
{
  Serial.begin(115200);
  RW.begin();
}

void loop() 
{  
  uint8_t state = RW.Button.read(0);
  printf("Button State %d\n", state);
  delay(500);
}
