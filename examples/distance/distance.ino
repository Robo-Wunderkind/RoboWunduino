#include "robowunderkind.h"

RoboWunderkind RW = RoboWunderkind();

void setup() 
{
  Serial.begin(115200);
  RW.begin();
}

void loop() 
{  
  uint8_t distance = RW.Ultrasonic.read_distance(0);
  printf("Distance %d\n", distance);
  delay(500);
}
