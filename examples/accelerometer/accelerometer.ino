#include "robowunderkind.h"

RoboWunderkind RW = RoboWunderkind();

float acc_x, acc_y, acc_z = 0;
float gyro_x, gyro_y, gyro_z = 0; 
uint8_t imu_index = 0;

void setup() 
{
  Serial.begin(115200);
  RW.begin();
}

void loop() 
{  
  RW.IMU.read_accelerometer(imu_index, &acc_x, &acc_y, &acc_z);
  RW.IMU.read_gyroscope(imu_index, &gyro_x, &gyro_y, &gyro_z);
  printf("Gravitational Acceleration X: %0.2f Y: %0.2f Z: %0.2f from IMU sensor %d\n", acc_x, acc_y, acc_z, imu_index);
  printf("Rotational Acceleration X: %0.2f Y: %0.2f Z: %0.2f from IMU sensor %d\n", gyro_x, gyro_y, gyro_z, imu_index);
  delay(500);
}
