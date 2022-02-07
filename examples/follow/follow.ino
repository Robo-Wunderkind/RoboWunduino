#include "robowunderkind.h"

RoboWunderkind RW = RoboWunderkind();

#define Ultrasonic_Right 0
#define Ultrasonic_Left  1

#define LED_Right 1
#define LED_Left  0

#define Motor_Right 1
#define Motor_Left  0

float target_distance = 10.0;
float active_distance = 75.0;
float turning_delta = 2.5;
float avgR = 0;
float avgL = 0;
uint8_t max_speed = 100;
uint8_t min_speed = 30;

void setup() 
{
  Serial.begin(115200);
  RW.begin();

}

void sample_distance_readings(float *avgR, float *avgL)
{
  *avgR = 0;
  *avgL = 0;
  for(uint8_t i = 0; i < 10; i++)
  {
    uint8_t distanceR = RW.Ultrasonic.read(Ultrasonic_Right);
    uint8_t distanceL = RW.Ultrasonic.read(Ultrasonic_Left);
    //printf("Distance R %d    Distance L %d\n", distanceR, distanceL);
    *avgR += distanceR;
    *avgL += distanceL;
  }
  *avgR = *avgR/10.0;
  *avgL = *avgL/10.0;
}

void follow()
{
  int8_t delta = 0;
  uint8_t distance_error = 0;
  uint8_t speed;
  
  sample_distance_readings(&avgR, &avgL);
  delta = avgR - avgL;
  //printf("Average Distance R %0.2f   Average Distance L %0.2f\n", avgR, avgL);

  if(avgR <= active_distance && avgR >= target_distance && avgL <= active_distance && avgL >= target_distance)
  {
    if(abs(delta) < turning_delta && avgR > target_distance)
    {
      // Forward
      speed = max_speed*(avgR/active_distance);
      if(speed < min_speed) speed = min_speed;
      RW.Motor.speed(Motor_Right, -speed);
      RW.Motor.speed(Motor_Left, speed);
      RW.LED.rgb(255,255,0,LED_Right);
      RW.LED.rgb(255,255,0,LED_Left);
    }
    else if(delta >= turning_delta && avgR > target_distance)
    {
      // Right
      speed = max_speed*(abs(delta)/5);
      if(speed < min_speed) speed = min_speed;
      RW.Motor.speed(Motor_Right, -speed);
      RW.Motor.speed(Motor_Left, 0);
      RW.LED.rgb(255,255,0,LED_Right);
      RW.LED.rgb(0,0,0,LED_Left);
    }
    else if(delta <= -turning_delta && avgL > target_distance)  
    {
      // Left
      speed = max_speed*(abs(delta)/5);
      if(speed < min_speed) speed = min_speed;
      RW.Motor.speed(Motor_Right, 0);
      RW.Motor.speed(Motor_Left, speed);
      RW.LED.rgb(0,0,0,LED_Right);
      RW.LED.rgb(255,255,0,LED_Left);
    }
  }
  else if(avgR <= target_distance && avgL <= target_distance)
  {
    // Reached Target
    RW.Motor.stop_all();
    RW.LED.rgb( 0, 255, 0, LED_Right);
    RW.LED.rgb( 0, 255, 0, LED_Left);
  }
  else
  {
    // Measurements out of range
    RW.Motor.stop_all();
    RW.LED.rgb( 255, 0, 0, LED_Right);
    RW.LED.rgb( 255, 0, 0, LED_Left);
  }
}

void loop() 
{  
  follow();
}
