#include "robowunderkind.h"

uint8_t R = 0;
uint8_t G = 255;
uint8_t B = 0;

RoboWunderkind RW = RoboWunderkind();

void setup() 
{
  Serial.begin(115200);
  RW.begin();
}

void loop() 
{  
  //uint8_t r, uint8_t g, uint8_t b, uint8_t blinks, float frequency, uint8_t module_num
  RW.LED.blink(R, G, B, 5, 1, 0);
  RW.wait_for_all_actions();
  delay(5000);
}
