#include "robowunderkind.h"

uint8_t R = 0;
uint8_t G = 255;
uint8_t B = 0;

uint8_t blinks = 5;
uint8_t frequency = 1;

uint8_t led_index = 0;

RoboWunderkind RW = RoboWunderkind();

void setup() 
{
  Serial.begin(115200);
  RW.begin();
}

void loop() 
{  
  RW.LED.blink(led_index, R, G, B, blinks, frequency);
  RW.wait_for_all_actions();
  delay(5000);
}
