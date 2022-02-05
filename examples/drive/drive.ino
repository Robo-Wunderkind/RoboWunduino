#include "robowunderkind.h"

RoboWunderkind RW = RoboWunderkind();

bool motors_used[MAX_MOTORS] = {ENABLE,ENABLE,DISABLE,DISABLE,DISABLE,DISABLE};            
bool directions[MAX_MOTORS] = {RIGHT,LEFT,LEFT,LEFT,LEFT,LEFT};  

void setup() 
{
  Serial.begin(115200);
  RW.begin();
  RW.Motor.config(motors_used, directions);
}

void loop() 
{  
  RW.Motor.forward(100, 100);
  RW.wait_for_all_actions();
  delay(500);
  RW.Motor.backward(100, 100);
  RW.wait_for_all_actions();
  delay(500);
  RW.Motor.turn(100, 720);
  RW.wait_for_all_actions();
  delay(500);
}
