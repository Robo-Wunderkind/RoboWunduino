#include "robowunderkind.h"

RoboWunderkind RW = RoboWunderkind();

// Using some of the predefined images from the Display Class
const uint8_t test_animation[5][32] = {ROBO_LOGO, CHECK_MARK, STOP, GO, UP};
const uint8_t test_animation_size = sizeof(test_animation) / sizeof(test_animation[0]); 

void setup() 
{
  Serial.begin(115200);
  RW.begin();
  
}

void loop() 
{  
  RW.Display.custom_animation(0, test_animation, test_animation_size, 1, 1000);
  RW.wait_for_all_actions();
  RW.Display.animation(0, 1, 0);
}
