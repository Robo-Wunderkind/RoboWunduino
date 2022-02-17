#include "robowunderkind.h"

RoboWunderkind RW = RoboWunderkind();

void setup() 
{
  Serial.begin(115200);
  RW.begin();
}

void loop() 
{  
  printf("Temperature = %0.2f C\n", RW.Weather.read_temp_c(0)); 
  printf("Temperature = %0.2f F\n", RW.Weather.read_temp_f(0)); 
  printf("Pressure = %0.2f mb\n", RW.Weather.read_pressure_mb(0)); 
  printf("Pressure = %0.2f kpa\n", RW.Weather.read_pressure_kpa(0)); 
  printf("Analog Temperature = %0.2f C\n", RW.Weather.read_analog_temp(0)); 
  printf("Relative Humidity = %0.2f %\n\n", RW.Weather.read_humidity_rh(0)); 

  printf("TVOC = %d ppb\n", RW.Weather.read_tvoc(0)); 
  printf("eCO2 = %d ppm\n", RW.Weather.read_eco2(0)); 
  printf("H2 = %d raw\n", RW.Weather.read_h2(0)); 
  printf("Ethanol = %d raw\n\n", RW.Weather.read_ethanol(0)); 

  printf("Altitude = %0.2fm \n", RW.Weather.read_altitude_m(0, 1030.0)); 
  delay(500);
}
