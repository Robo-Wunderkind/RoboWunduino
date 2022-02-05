#include "robowunderkind.h"

uint8_t R = 0;
uint8_t G = 0;
uint8_t B = 0;

RoboWunderkind RW = RoboWunderkind();

void change_colour()
{
  randomSeed(analogRead(39));
  uint8_t New_R = random(255);
  uint8_t New_G = random(255);
  uint8_t New_B = random(255);

  while(R != New_R && G != New_G && B != New_B)
  {
    if(R > New_R) R--;
    else if(R < New_R) R++;

    if(G > New_G) G--;
    else if(G < New_G) G++;

    if(B > New_B) B--;
    else if(B < New_B) B++;
    delay(20);
    RW.LED.rgb(R, G, B, 0);
  }
}

void setup() 
{
  Serial.begin(115200);
  RW.begin();
}

void loop() 
{  
  change_colour();
  delay(500);
}
