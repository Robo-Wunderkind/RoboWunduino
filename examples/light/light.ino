#include "robowunderkind.h"

RoboWunderkind RW = RoboWunderkind();

void setup() 
{
  Serial.begin(115200);
  RW.begin();
}

void loop() 
{  
  uint8_t light = RW.Light.read(0);
  printf("Light Level %d\n", light);
  delay(500);
}
