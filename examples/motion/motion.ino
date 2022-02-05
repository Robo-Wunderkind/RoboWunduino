#include "robowunderkind.h"

RoboWunderkind RW = RoboWunderkind();

void setup() 
{
  Serial.begin(115200);
  RW.begin();
}

void loop() 
{  
  uint8_t motion = RW.Motion.read(0);
  printf("Motion State %d\n", motion);
  delay(500);
}
