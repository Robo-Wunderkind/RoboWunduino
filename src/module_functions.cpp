/*
 * modules.c
 *   ----------  GENERAL DESCRIPTION:  ----------
 *
 *  set module parameters, read modules sensorvalues, set/reset actions and triggers.
 */
 
#include "i2c_bus.h"
#include "module_handler.h"
#include "module_functions.h"

uint8_t matrix_data_string[33];
uint8_t matrix_string_add = 0;

bool write_to_module(uint8_t module_type, uint8_t address, uint8_t module_index, uint8_t *data, uint8_t size_d)
{
  if (check_if_attached_from_index(module_type, module_index))
  {
    return i2c_write_module(address, data, size_d);
  }
  else 
  {
    printf("Can`t send I2C-command, address 0X%02X not attached, index %d\n", address, module_index);
  }
  return false;
}

uint8_t get_eeprom_module_id(uint8_t address)
{
  uint8_t message_bytes = 25;

  uint8_t data_rd[message_bytes];
  if(i2c_read_module_data(address, data_rd, message_bytes) != 0) return data_rd[24];
  else return 0xff;
}

bool set_eeprom_module_id(uint8_t address, uint8_t id)
{
  uint8_t message_bytes = 3;
  uint8_t data[message_bytes];

  data[0] = 0x20;
  data[1] = id;

  vTaskDelay(100 / portTICK_PERIOD_MS); 
  i2c_write_module(address, data, message_bytes);
  vTaskDelay(100 / portTICK_PERIOD_MS); 
  if(get_eeprom_module_id(address) == id)
  {
    //printf("I2C: Assigned new eeprom id to: %d\n", id);
    return true;
  }
  else 
  {
    //printf("I2C: Failed to assign new eeprom id to: %d\n", id);
    return false;
  }
}

bool set_zero_servo(uint8_t module_index)
{
  uint8_t data[2];
  data[0] = 0x12;
  return write_to_module(MODULE_SERVO, (SERVO1_ADD + module_index), module_index, data, 2);
}

bool lock_hinge(uint8_t module_index)
{
  uint8_t data[2];
  data[0] = 0x12;
  data[1] = 0x00;
  return write_to_module(MODULE_HINGE, (HINGE1_ADD + module_index), module_index, data, 2);
}

bool set_hinge_torque(uint8_t module_index, int8_t speed)
{
  uint8_t data[3];
  float pwm = ((float)-speed/100.0)*127;
  data[0] = 0x10;
  data[1] = (int8_t)pwm;
  return write_to_module(MODULE_HINGE, (HINGE1_ADD + module_index), module_index, data, 3);
}

bool set_hinge_position(uint8_t module_index, int8_t angle)
{
  uint8_t data[3];
  uint8_t set_angle = 0;

  set_angle = angle + 90;
  data[0] = 0x11;
  data[1] = set_angle;
  return write_to_module(MODULE_HINGE, (HINGE1_ADD + module_index), module_index, data, 3);
}

bool lock_claw(uint8_t module_index)
{
  uint8_t data[2];
  data[0] = 0x12;
  data[1] = 0x00;
  return write_to_module(MODULE_CLAW, (CLAW1_ADD + module_index), module_index, data, 2);
}

bool set_claw_torque(uint8_t module_index, int8_t speed)
{
  uint8_t data[3];
  float pwm = ((float)-speed/100.0)*127;
  data[0] = 0x10;
  data[1] = (int8_t)pwm;
  return write_to_module(MODULE_CLAW, (CLAW1_ADD + module_index), module_index, data, 3);
}

bool claw_open_close(uint8_t module_index, bool open_close)
{
  uint8_t data[3];
  if(open_close) data[0] = 0x14;
  else data[0] = 0x13;
  data[1] = 100;
  return write_to_module(MODULE_CLAW, (CLAW1_ADD + module_index), module_index, data, 3);
}

bool set_motor_pwm(uint8_t module_index, int8_t speed)
{
  uint8_t data[3];
  float pwm = ((float)speed/100.0)*127;
  data[0] = 0x10;
  data[1] = (int8_t)pwm;
  return write_to_module(MODULE_MOTOR, (MOTOR1_ADD + module_index), module_index, data, 3);
}

bool set_motor_speed(uint8_t module_index, int8_t speed)
{
  uint8_t data[6];
  uint8_t wheel_diameterHSB = WHEEL_DIAMETER >> 8;
  uint8_t wheel_diameterLSB = WHEEL_DIAMETER >> 0;

  int16_t speed_ticks = (MAXVEL*(speed/100.0));
  if(speed_ticks < MINVEL && speed_ticks > 0) speed_ticks = MINVEL;
  if(speed_ticks > -MINVEL && speed_ticks < 0) speed_ticks = -MINVEL;

  int8_t speed_HSB = speed_ticks >> 8;
  int8_t speed_LSB = speed_ticks >> 0;
  
  data[0] = 0x13;
  data[1] = speed_HSB;
  data[2] = speed_LSB;
  data[3] = wheel_diameterHSB;
  data[4] = wheel_diameterLSB;
  return write_to_module(MODULE_MOTOR, (MOTOR1_ADD + module_index), module_index, data, 6);
}

bool set_motor_angle(uint8_t module_index, int16_t angle, uint8_t direction)
{
  uint8_t data[8];
  angle*=1.04;
  int16_t speed = 150;
  if(direction == 0) speed *= -1;
  data[0] = 0x15;
  data[1] = (int8_t)(speed >> 8);
  data[2] = (int8_t)(speed >> 0);
  data[3] = 0;
  data[4] = 0x59;
  data[5] = (int8_t)(angle >> 8);
  data[6] = (int8_t)(angle >> 0);
  return write_to_module(MODULE_MOTOR, (MOTOR1_ADD + module_index), module_index, data, 8);
}

bool adjust_motor_speed(uint8_t module_index, uint8_t speedHSB, uint8_t speedLSB, uint8_t wheel_diameterHSB, uint8_t wheel_diameterLSB)
{
  uint8_t data[6];
  int16_t speed = ((int16_t)speedHSB << 8 | (int16_t)speedLSB);
  speed = speed * MOTOR_SPEED_CONVERSION_RATE;

  data[0] = 0x16;
  data[1] = (int8_t)(speed >> 8);
  data[2] = (int8_t)(speed >> 0);
  data[3] = wheel_diameterHSB;
  data[4] = wheel_diameterLSB;
  return write_to_module(MODULE_MOTOR, (MOTOR1_ADD + module_index), module_index, data, 6);
}

bool set_servo_position(uint8_t module_index, int8_t position)
{
  uint8_t data[4];
  uint16_t unsigned_pos = position * 1.5f + 120;
  unsigned_pos = (unsigned_pos * (-1)) + 300;

  //printf("Setting Servo to: %d\n", unsigned_pos);

  data[0] = 0x10;
  data[1] = (uint8_t)(unsigned_pos >> 8);
  data[2] = (uint8_t)(unsigned_pos >> 0);
  return write_to_module(MODULE_SERVO, (SERVO1_ADD + module_index), module_index, data, 4);
}

bool set_rgb_color(uint8_t module_index, uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t data[6];
  data[0] = 0x10;
  data[1] = r;
  data[2] = g;
  data[3] = b;
  data[4] = 0;
  return write_to_module(MODULE_RGB, (RGB1_ADD + module_index), module_index, data, 6);
}

bool calibrate_linetracker(uint8_t module_index)
{
  uint8_t data[2];
  data[0] = 0x13;
  return write_to_module(MODULE_LINETRACKER, (LINETRACKER1_ADD + module_index), module_index, data, 2);
}

bool set_matrix_leds(uint8_t module_index, uint64_t leds)
{
  uint8_t data[18];
  data[0] = 0x10;
  data[2] =   (uint8_t)(leds >> 32);
  data[4] =   (uint8_t)(leds >> 40);
  data[6] =   (uint8_t)(leds >> 48);
  data[8] =   (uint8_t)(leds >> 56);
  data[10] =  (uint8_t)(leds >> 0);
  data[12] =  (uint8_t)(leds >> 8);
  data[14] =  (uint8_t)(leds >> 16);
  data[16] =  (uint8_t)(leds >> 24);

  data[2] =   ((data[2] & 0x0F)<<4 | (data[2] & 0xF0)>>4);
  data[4] =   ((data[4] & 0x0F)<<4 | (data[4] & 0xF0)>>4);
  data[6] =   ((data[6] & 0x0F)<<4 | (data[6] & 0xF0)>>4);
  data[8] =   ((data[8] & 0x0F)<<4 | (data[8] & 0xF0)>>4);
  data[10] =  ((data[10] & 0x0F)<<4 | (data[10] & 0xF0)>>4);
  data[12] =  ((data[12] & 0x0F)<<4 | (data[12] & 0xF0)>>4);
  data[14] =  ((data[14] & 0x0F)<<4 | (data[14] & 0xF0)>>4);
  data[16] =  ((data[16] & 0x0F)<<4 | (data[16] & 0xF0)>>4);
  data[1] =   0;
  data[3] =   0;
  data[5] =   0;
  data[7] = 0;
  data[9] =   0;
  data[11] =  0;
  data[13] =  0;
  data[15] =  0;
  return write_to_module(MODULE_MATRIX, (MATRIX1_ADD + module_index), module_index, data, 18);
}

/*
================================================================== READ FUNCTIONS =============================================================================
*/

bool read_ultrasonic_distance(uint8_t module_index, float *distance_cm)
{
  uint8_t message_bytes = 13;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(ULTRASONIC1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    i2c_read_module_data(ULTRASONIC1_ADD + module_index, data_rd, message_bytes);
    if(data_rd[1] == 255 && data_rd[2] == 255) return 255;
    uint16_t distance = ((uint16_t)data_rd[1] << 8) | (uint16_t)data_rd[2];
    *distance_cm = distance * ULTRASONIC_DISTANCE_CONVERSION_RATE;
    return true;
  }
  else
  {
    //printf("Can`t send I2C-command, ultrasonic %d not attached\n", module_index + 1);
    *distance_cm = 0;
    return false;
  }
}

bool read_ultrasonic_volume(uint8_t module_index, float *vol)
{
  uint8_t message_bytes = 13;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(ULTRASONIC1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    uint16_t volume = 0;
    uint8_t data_rd[message_bytes];
    for (uint8_t i = 0; i < 5; i++)
    {
      i2c_read_module_data(ULTRASONIC1_ADD + module_index, data_rd, message_bytes);
      volume += ((uint16_t)data_rd[4] << 8) | data_rd[5];
    }
    volume /= 5;
    *vol = volume * SOUNDLEVEL_CONVERSION_RATE;
    return true;
  }
  else
  {
    //printf("Can`t send I2C-command, ultrasonic %d not attached\n", module_index + 1);
    *vol = 0;
    return false;
  }
}

bool read_motor_distance(uint8_t module_index, float *distance_in_cm)
{
  uint8_t message_bytes = 6;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(MOTOR1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    uint32_t distance = (data_rd[1] << 24) | (data_rd[2] << 16) | (data_rd[3] << 8) | data_rd[4];
    *distance_in_cm = distance;
    return true;
  }
  else
  {
    //printf("Can`t send I2C-command, motor %d not attached\n", module_index + 1);
    *distance_in_cm = 0;
    return false;
  }
}

bool read_pir_state(uint8_t module_index, uint8_t *pir_state)
{
  uint8_t message_bytes = 3;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(PIR1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    *pir_state = data_rd[1];
    return true;
  }
  else
  {
    //printf("Can`t send I2C-command, pir %d not attached\n", module_index + 1);
    *pir_state = 0;
    return false;
  }
}

bool read_claw_proximity(uint8_t module_index, uint8_t *prox_state)
{
  uint8_t message_bytes = 16;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(CLAW1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    *prox_state = data_rd[13];
    return true;
  }
  else
  {
    //printf("Can`t send I2C-command, claw %d not attached\n", module_index + 1);
    *prox_state = 0;
    return false;
  }
}

bool read_knob(uint8_t module_index, uint16_t *knob_reading)
{
  uint8_t message_bytes = 4;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(KNOB1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    *knob_reading = ((uint16_t)data_rd[1] << 8) | data_rd[2];
    return true;
  }
  else
  {
    //printf("Unable to send I2C-command, knob %d not attached\n", module_index + 1);
    *knob_reading = 0;
    return false;
  }
}

bool read_lightsensor(uint8_t module_index, float *light)
{
  uint8_t message_bytes = 4;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(LIGHTSENSOR1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    uint16_t lightlevel = ((uint16_t)data_rd[1] << 8) | data_rd[2];
    *light = (float)lightlevel * LIGHTLEVEL_CONVERSION_RATE;
    return true;
  }
  else
  {
    printf("Unable to send I2C-command, lightsensor %d not attached\n", module_index + 1);
    *light = 0;
    return false;
  }
}

bool read_button_state(uint8_t module_index, uint8_t *button_state)
{
  uint8_t message_bytes = 5;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(BUTTON1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    *button_state = data_rd[1];
    return true;
  }
  else
  {
    printf("Unable to send I2C-command, button %d not attached\n", module_index + 1);
    *button_state = 0;
    return false;
  }
}

bool read_claw_potentiometer(uint8_t module_index, uint16_t *potentiometer)
{
  uint8_t message_bytes = 16;
  if (check_if_attached_from_index(CLAW, module_index)) 
  {
    uint8_t data_rd[message_bytes];
    i2c_read_module_data(CLAW1_ADD + module_index, data_rd, message_bytes);
    *potentiometer = ((uint16_t)data_rd[1] << 8) | data_rd[2];
    return true;
  }
  else
  {
    //printf("Unable to send I2C-command, claw %d not attached\n", module_index + 1);
    *potentiometer = 0;
    return false;
  }
}

bool read_linetracker_sensorvalues(uint8_t module_index, linetracker_sensorvalues *values, uint16_t sensor_saturation)
{
  uint8_t message_bytes = 19;
  uint8_t data_rd[message_bytes];
  if ( i2c_read_module_data(LINETRACKER1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    for (uint8_t i = 0; i < message_bytes; i++)
    {
      data_rd[i] = 0;
    }
   
    if((data_rd[1] << 8 | data_rd[2]) < sensor_saturation && (data_rd[3] << 8 | data_rd[4]) < sensor_saturation && (data_rd[5] << 8 | data_rd[6]) < sensor_saturation)
    {
      values->l = data_rd[1] << 8 | data_rd[2];
      values->c = data_rd[3] << 8 | data_rd[4];
      values->r = data_rd[5] << 8 | data_rd[6];
    }
    values->edge_l = data_rd[7];
    values->edge_c = data_rd[8];
    values->edge_r = data_rd[9];

    values->bin_l = data_rd[16];
    values->bin_c = data_rd[17];
    values->bin_r = data_rd[18];

    return true;
  }
  else
  {
    //printf("Unable to send I2C-command, linetracker %d not attached\n", module_index + 1);
    return false;
  }
}

bool read_gyro(uint8_t module_index, float *gyrox, float *gyroy, float *gyroz)
{
  uint8_t message_bytes = 21;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(ACCELEROMETER1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    int16_t x = (((uint16_t)data_rd[7]<<8) | data_rd[8]); 
    int16_t y = (((uint16_t)data_rd[9]<<8) | data_rd[10]);
    int16_t z = (((uint16_t)data_rd[11]<<8) | data_rd[12]);

    *gyrox = (float)x/GYROSCOPE_SENSITIVITY;
    *gyroy = (float)y/GYROSCOPE_SENSITIVITY;
    *gyroz = (float)z/GYROSCOPE_SENSITIVITY;
    return true;
  }
  else
  {
    *gyrox = 0;
    *gyroy = 0;
    *gyroz = 0;
    return false;
  }
}

bool read_accelerometer_values(uint8_t module_index, float *accx, float *accy, float *accz)
{
  uint8_t message_bytes = 21;
  uint8_t data_rd[message_bytes];
  if (i2c_read_module_data(ACCELEROMETER1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    int16_t x = (((uint16_t)data_rd[1]<<8) | data_rd[2]); 
    int16_t y = (((uint16_t)data_rd[3]<<8) | data_rd[4]);
    int16_t z = (((uint16_t)data_rd[5]<<8) | data_rd[6]);

    *accx = (float)x/ACC_4G_CONV_G;
    *accy = (float)y/ACC_4G_CONV_G;
    *accz = (float)z/ACC_4G_CONV_G;
    return true;
  }
  else
  {
    *accx = 0;
    *accy = 0;
    *accz = 0;
    return false;
  }
}

bool read_accelerometer_state(uint8_t module_index)
{
  uint8_t message_bytes = 21;
  uint8_t data_rd[message_bytes];
  uint8_t acc_values = 0;
  if (i2c_read_module_data(ACCELEROMETER1_ADD + module_index, data_rd, message_bytes) != 0)
  {
      if (data_rd[19] == 1) acc_values |= (1U << 0);
      if (data_rd[18] == 1) acc_values |= (1U << 1);
      if (data_rd[20] == 1) acc_values |= (1U << 2);
    return acc_values;
  }
  else
  {
    //printf("Unable to send I2C-command, accelerometer %d not attached\n", module_index + 1);
    return false;
  }
}

int32_t twos_comp(uint32_t num, uint8_t bits)
{
  int32_t power = pow(2, bits);
  if(num > power/2.0) num = num - power;
  return num;
}

bool read_gas_sensor(uint8_t module_index, uint16_t *tvoc, uint16_t *eco2, uint16_t *h2, uint16_t *ethanol)
{
  uint8_t message_bytes = 14;
  uint8_t data_rd[message_bytes];

  if (i2c_read_module_data(WEATHER1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    *tvoc =    (((uint16_t)data_rd[5]<<8) | data_rd[6]);
    *tvoc = twos_comp((int32_t)*tvoc, 16);
    *eco2 =    (((uint16_t)data_rd[7]<<8) | data_rd[8]);
    *eco2 = twos_comp((int32_t)*eco2, 16);
    *h2 =      (((uint16_t)data_rd[9]<<8) | data_rd[10]);
    *h2 = twos_comp((int32_t)*h2, 16);
    *ethanol = (((uint16_t)data_rd[11]<<8) | data_rd[12]);
    *ethanol = twos_comp((int32_t)*ethanol, 16);
    return true;
  }
  else
  {
    //printf("Unable to send I2C-command, weather %d not attached\n", module_index + 1);
    *tvoc = 0;
    *eco2 = 0;
    *h2 = 0;
    *ethanol = 0;
    return false;
  }
}

bool read_analog_TempHum(uint8_t module_index, uint16_t *temperature, uint16_t *humidity)
{
  uint8_t message_bytes = 6;
  uint8_t data_rd[message_bytes];

  if (i2c_read_module_data(WEATHER1_ADD + module_index, data_rd, message_bytes) != 0)
  { 
    *humidity =     (((uint16_t)data_rd[1]<<8) | data_rd[2]);
    *temperature =  (((uint16_t)data_rd[3]<<8) | data_rd[4]);
    return true;
  }
  else
  {
    //printf("Unable to send I2C-command, weather %d not attached\n", module_index + 1);
    *humidity = 0;
    *temperature = 0;
    return false;
  }
}

bool read_spl_coef(uint8_t module_index, int16_t *c0, int16_t *c1, int32_t *c00, int16_t *c01, int32_t *c10, int16_t *c11, int16_t *c20, int16_t *c21, int16_t *c30)
{
  uint8_t message_bytes = 38;
  uint8_t data_rd[message_bytes];

  if (i2c_read_module_data(WEATHER1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    *c0 = (((uint16_t)data_rd[19]<<4) | ((uint16_t)data_rd[20]>>4));
    *c0 = twos_comp((uint32_t)*c0, 12);
    *c1 = (((uint16_t)data_rd[20]&0xF) << 8) | (uint16_t)data_rd[21];
    *c1 = twos_comp((uint32_t)*c1, 12);
    *c00 = ((uint32_t)data_rd[22] << 12) | ((uint32_t)data_rd[23] << 4) | ((uint32_t)data_rd[24] >> 4);
    *c00 = twos_comp((uint32_t)*c00, 20);
    *c10 = (((uint32_t)data_rd[24]&0xF) << 16) | ((uint32_t)data_rd[25] << 8) | ((uint32_t)data_rd[26]);
    *c10 = twos_comp((uint32_t)*c10, 20);
    *c01 = ((uint16_t)data_rd[27] << 8) | (uint16_t)data_rd[28];
    *c01 = twos_comp((uint32_t)*c01, 16);
    *c11 = ((uint16_t)data_rd[29] << 8) | (uint16_t)data_rd[30];
    *c11 = twos_comp((uint32_t)*c11, 16);
    *c20 = ((uint16_t)data_rd[31] << 8) | (uint16_t)data_rd[32];
    *c20 = twos_comp((uint32_t)*c20, 16);
    *c21 = ((uint16_t)data_rd[33] << 8) | (uint16_t)data_rd[34];
    *c21 = twos_comp((uint32_t)*c21, 16);
    *c30 = ((uint16_t)data_rd[35] << 8) | (uint16_t)data_rd[36];
    *c30 = twos_comp((uint32_t)*c30, 16);
/*
    for(uint8_t i = 0; i < 18; i ++)
    {
      printf("Byte %d = %d \n", 19+i, data_rd[19+i]);
    }
    printf("C0 = %d\n C1 = %d\n C00 = %d\n C10 = %d\n C01 = %d\n C11 = %d\n C20 = %d\n C21 = %d\n C30 = %d\n", *c0, *c1, *c00, *c10, *c01, *c11, *c20, *c21, *c30);
    */
    return true;
  }
  else
  {
    //printf("Unable to send I2C-command, weather %d not attached\n", module_index + 1);
    *c0 = 0;
    *c1 = 0;
    *c00 = 0;
    *c10 = 0;
    *c01 = 0;
    *c11 = 0;
    *c20 = 0;
    *c21 = 0;
    *c30 = 0;
    return false;
  }
}

bool read_spl(uint8_t module_index, float *temperature_sc, float *pressure_sc)
{
  uint8_t message_bytes = 22;
  uint8_t data_rd[message_bytes];
  int32_t temperature_in = 0;
  int32_t pressure_in = 0;

  //float SPL_SC[8] = {524288.0, 1572864.0, 3670016.0, 7864320.0, 253952.0, 516096.0, 1040384.0, 2088960.0};

  if (i2c_read_module_data(WEATHER1_ADD + module_index, data_rd, message_bytes) != 0)
  {
    temperature_in = ((uint32_t)data_rd[13]<<16) | ((uint32_t)data_rd[14]<<8) | (uint32_t)data_rd[15];
    *temperature_sc = twos_comp((uint32_t)temperature_in, 24);

    pressure_in = ((uint32_t)data_rd[16]<<16) | ((uint32_t)data_rd[17]<<8) | (uint32_t)data_rd[18];
    pressure_in = twos_comp((uint32_t)pressure_in, 24);

    *temperature_sc = ((float)temperature_in)/7864320.0;
    *pressure_sc = ((float)pressure_in)/7864320.0;
    return true;
  }
  else
  {
    //printf("Unable to send I2C-command, weather %d not attached\n", module_index + 1);
    *temperature_sc = 0;
    *pressure_sc = 0;
    return false;
  }
}

//================================================================== SET ACTION FUNCTIONS =============================================================================


bool set_motor_action(uint8_t module_index, int8_t speed, uint8_t wheel_diameterHSB, uint8_t wheel_diameterLSB, uint8_t distanceHSB, uint8_t distanceLSB)
{
  uint8_t data[8];
  int16_t distance = ((int16_t)distanceHSB << 8 | (int16_t)distanceLSB);

  int16_t speed_ticks = (MAXVEL*(speed/100.0));
  if(speed_ticks < MINVEL && speed_ticks > 0) speed_ticks = MINVEL;
  if(speed_ticks > -MINVEL && speed_ticks < 0) speed_ticks = -MINVEL;

  int8_t speed_HSB = speed_ticks >> 8;
  int8_t speed_LSB = speed_ticks >> 0;
  
  data[0] = 0x14;
  data[1] = (int8_t)(speed_ticks >> 8);
  data[2] = (int8_t)(speed_ticks >> 0);
  data[3] = wheel_diameterHSB;
  data[4] = wheel_diameterLSB;
  data[5] = (int8_t)(distance >> 8);
  data[6] = (int8_t)(distance >> 0);
  return write_to_module(MODULE_MOTOR, (MOTOR1_ADD + module_index), module_index, data, 8);
}

bool set_claw_action(uint8_t module_index, bool open_close)
{
  uint8_t data[4];
  if(open_close) data[0] = 0x13;
  else data[0] = 0x14;
  data[1] = 0x64;
  return write_to_module(MODULE_CLAW, (CLAW1_ADD + module_index), module_index, data, 4);
}

bool set_servo_action(uint8_t module_index, uint8_t positionHSB, uint8_t positionLSB)
{
  uint8_t data[4];
  int16_t signed_pos = - ((int16_t)positionHSB << 8 | (int16_t)positionLSB);
  signed_pos += 180;

  data[0] = 0x10;
  data[1] = (int8_t)(signed_pos >> 8);
  data[2] = (int8_t)(signed_pos >> 0);
  return write_to_module(MODULE_SERVO, (SERVO1_ADD + module_index), module_index, data, 4);
}

bool set_rgb_action(uint8_t module_index, uint8_t r, uint8_t g, uint8_t b, uint8_t timeHSB, uint8_t timeLSB, uint8_t blinks , uint8_t pulse)
{
  uint8_t data[9];
  data[1] = r;
  data[2] = g;
  data[3] = b;
  data[4] = 0;
  data[5] = timeHSB;
  data[6] = timeLSB;

  if(pulse == 1 && blinks > 0)
  {
    data[0] = 0x14;
    data[4] = timeHSB;
    data[5] = timeLSB;
    data[6] = blinks;
  }
  else if (blinks == 0)
  {
    data[0] = 0x11;
    data[7] = 0;
  }
  else
  {
    data[0] = 0x12;
    data[7] = blinks;
  }
  return write_to_module(MODULE_RGB, (RGB1_ADD + module_index), module_index, data, 9);
}

bool set_matrix_action(uint8_t module_index, uint64_t leds, uint8_t timeHSB, uint8_t timeLSB)
{
  uint8_t data[20];
  data[0] = 0x11;
  data[2] =   (uint8_t)(leds >> 32);
  data[4] =   (uint8_t)(leds >> 40);
  data[6] =   (uint8_t)(leds >> 48);
  data[8] =   (uint8_t)(leds >> 56);
  data[10] =  (uint8_t)(leds >> 0);
  data[12] =  (uint8_t)(leds >> 8);
  data[14] =  (uint8_t)(leds >> 16);
  data[16] =  (uint8_t)(leds >> 24);

  data[2] =   ((data[2] & 0x0F)<<4 | (data[2] & 0xF0)>>4);
  data[4] =   ((data[4] & 0x0F)<<4 | (data[4] & 0xF0)>>4);
  data[6] =   ((data[6] & 0x0F)<<4 | (data[6] & 0xF0)>>4);
  data[8] =   ((data[8] & 0x0F)<<4 | (data[8] & 0xF0)>>4);
  data[10] =  ((data[10] & 0x0F)<<4 | (data[10] & 0xF0)>>4);
  data[12] =  ((data[12] & 0x0F)<<4 | (data[12] & 0xF0)>>4);
  data[14] =  ((data[14] & 0x0F)<<4 | (data[14] & 0xF0)>>4);
  data[16] =  ((data[16] & 0x0F)<<4 | (data[16] & 0xF0)>>4);

  data[1] =   0;
  data[3] =   0;
  data[5] =   0;
  data[7] = 0;
  data[9] =   0;
  data[11] =  0;
  data[13] =  0;
  data[15] =  0;

  data[17] = timeHSB;
  data[18] = timeLSB;

  return write_to_module(MODULE_MATRIX, (MATRIX1_ADD + module_index), module_index, data, 20);
}

bool matrix_set_string(void)
{
  uint8_t modules_enum = modules_address_to_enum(matrix_string_add);
  if (check_if_module_attached(modules_enum))
  {
    i2c_write_module(matrix_string_add, matrix_data_string, matrix_data_string[4] + 5);
  }
  //else printf("Can`t send I2C-command, matrix %d not attached\n", (matrix_string_add - MATRIX1_ADD));

  matrix_string_add = 0;
  for (uint8_t i = 0; i < 32; i++)
  {
    matrix_data_string[i] = 0;
  }
  return true;
}

bool set_matrix_string_action_0(uint8_t module_index, uint8_t text_orientation, uint8_t repeats, uint8_t scrolling_rate, uint8_t string_length, uint8_t *string)
{
  matrix_string_add = MATRIX1_ADD + module_index;

  uint8_t s_rate = 0;
  s_rate = scrolling_rate * 24;
  s_rate = 255 - s_rate;

  matrix_data_string[0] = 0x12;
  if    (text_orientation == 1) matrix_data_string[1] = 3;
  else if (text_orientation == 3) matrix_data_string[1] = 1;
  else  matrix_data_string[1] = text_orientation;

  if (repeats > 10)   matrix_data_string[2] = 255;
  else        matrix_data_string[2] = repeats;
  matrix_data_string[3] = s_rate;
  matrix_data_string[4] = string_length;

  for (uint8_t i = 0; ((i < string_length) && (i < 11)); i++)
  {
    if ((string[i] >= ' ') && (string[i] <= '~'))   matrix_data_string[i + 5] = string[i];    // char is between 0 ('space') and 126 ('~')
    else                      matrix_data_string[i + 5] = '?';
  }
  if (string_length <= 11) matrix_set_string();
  return true;
}

bool set_matrix_string_action_1(uint8_t string_length, uint8_t *string)
{
  for (uint8_t i = 0; ((i < string_length) && (i < 32)) ; i++)
  {
    if ((string[i] >= ' ') && (string[i] <= '~'))   matrix_data_string[i + 16] = string[i];   // char is between 0 ('space') and 126 ('~')
    else                      matrix_data_string[i + 16] = '?';
  }
  return matrix_set_string();
}

bool set_animation(uint8_t module_index, uint8_t animation_num, uint8_t repeats, uint8_t reverse, uint8_t orientation, uint8_t num_frames, uint8_t frame_rateH, uint8_t frame_rateL) 
{   // forward only, reverse = 0: forward and backward reverse = 1. num_frames only for custom animations (255)

  uint8_t data[9];
  data[0] = 0x13;
  data[1] = animation_num;
  data[2] = repeats;
  data[3] = reverse;
  data[4] = orientation;
  if (animation_num == 255){ // custom case
    data[5] = num_frames; 
    data[6] = frame_rateH;
    data[7] = frame_rateL;
  }
  else{ // pre-programmed
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
  }
  return write_to_module(MODULE_DISPLAY,(DISPLAY1_ADD + module_index), module_index, data, 9);
}

bool set_display_image(uint8_t module_index, uint8_t image, uint8_t orientation, uint8_t delayH, uint8_t delayL)
{
  uint8_t data[6];
  data[0] = 0x10;
  data[1] = image;
  data[2] = orientation;
  data[3] = delayH;
  data[4] = delayL;
  return write_to_module(MODULE_DISPLAY,(DISPLAY1_ADD + module_index), module_index, data, 6);
}

bool load_custom_display_image(uint8_t module_index, uint8_t frame_half, uint8_t rows[])
{
  uint8_t data[19];
  data[0] = 0x11;
  data[1] = frame_half;
  if(frame_half == 1 || frame_half == 2)
  {
    for(int i = 0; i < 16; i++)
    {
      data[i+2] = rows[i];
    }
  }
  return write_to_module(MODULE_DISPLAY,(DISPLAY1_ADD + module_index), module_index, data, 19);
}

bool set_custom_display_text(uint8_t module_index, uint8_t orientation, uint8_t total_length, uint8_t scrolling_rateH, uint8_t scrolling_rateL)
{
  uint8_t data[6];
  data[0] = 0x12;
  data[1] = orientation;
  data[2] = total_length;
  data[3] = scrolling_rateH;
  data[4] = scrolling_rateL;

  //printf("Orientation set: %d\n", orientation);
  //printf("Total Length set: %d\n", total_length);
  return write_to_module(MODULE_DISPLAY,(DISPLAY1_ADD + module_index), module_index, data, 6);
}

bool load_custom_display_animation(uint8_t module_index, uint8_t frame_index, uint8_t frame_half, uint8_t frame_rows[])
{
  if(frame_index >= 5)
  {
    //printf("Frame Index Overflow\n");
    return false;
  }

  uint8_t data[20];
  data[0] = 0x17;
  data[1] = frame_index;
  data[2] = frame_half;

  //printf("frame_index: %d\n", frame_index);
  //printf("frame_half: %d\n", frame_half);

  if(frame_half == 1 || frame_half == 2){
    for(int i = 0; i < 16; i++)
    {
      data[i+3] = frame_rows[i];
    }
  }
  return write_to_module(MODULE_DISPLAY,(DISPLAY1_ADD + module_index), module_index, data, 20);
}

bool load_custom_text(uint8_t module_index, char chars[], uint8_t starting_index, uint8_t length)
{
  uint8_t data[length + 4];
  data[0] = 0x18;
  data[1] = starting_index;
  data[2] = length;

  printf("%s\n", chars);

  for(int i = 0; i < length; i++)
  {
    data[i+3] = chars[i];
  }
  return write_to_module(MODULE_DISPLAY,(DISPLAY1_ADD + module_index), module_index, data, 20);
}

/*
================================================================== TRIGGER FUNCTIONS =============================================================================
*/

bool set_distance_trigger(uint8_t module_index, uint8_t condition, uint8_t distance_in_cm)
{
  uint8_t data[5];
  uint16_t converted_distance = distance_in_cm / ULTRASONIC_DISTANCE_CONVERSION_RATE;

  data[0] = 0x13;
  data[1] = (uint8_t)(converted_distance >> 8);
  data[2] = (uint8_t)(converted_distance >> 0);
  if (condition == 0)     data[3] = 2;    // less than
  else            data[3] = 1;    // more than
  return write_to_module(MODULE_ULTRASONIC, (ULTRASONIC1_ADD + module_index), module_index, data, 5); 
}

bool set_soundlevel_trigger(uint8_t module_index, uint8_t condition, uint8_t soundlevel)
{
  uint8_t data[5];
  uint16_t soundlevel_full = soundlevel / SOUNDLEVEL_CONVERSION_RATE;
  data[0] = 0x14;
  data[1] = (uint8_t)(soundlevel_full >> 8);
  data[2] = (uint8_t)(soundlevel_full >> 0);
  if (condition == 0)     data[3] = 2;    // less than
  else            data[3] = 1;    // more than
  return write_to_module(MODULE_ULTRASONIC, (ULTRASONIC1_ADD + module_index), module_index, data, 5); 
}

bool set_claw_trigger(uint8_t module_index, int8_t condition)
{
  uint8_t data[3];
  data[0] = 0x15;
  if (condition == 0)     data[1] = 1;    // OBJECT DETECTED
  else if (condition == 1)  data[1] = 2;    // OBJECT ABSENSE
  return write_to_module(MODULE_CLAW, (CLAW1_ADD + module_index), module_index, data, 3);
}

bool set_button_trigger(uint8_t module_index, int8_t condition)
{
  uint8_t data[4];
  if (condition > 0)                // clicked x times
  {
    data[0] = 0x10;
    data[1] = 6;
    data[2] = condition;            // condition holds number of clicks
  }
  else
  {
    data[0] = 0x10;
    if (condition == 0)     data[1] = 1;  // pressed
    else            data[1] = 3;  // released
  }
  return write_to_module(MODULE_BUTTON, (BUTTON1_ADD + module_index), module_index, data, 4);
}

bool set_light_trigger(uint8_t module_index, uint8_t condition, uint16_t lightlevel)
{
    uint8_t data[5];
    uint16_t lightlevel_full = lightlevel / LIGHTLEVEL_CONVERSION_RATE;
    data[0] = 0x10;
  data[1] = (uint8_t)(lightlevel_full >> 8);
  data[2] = (uint8_t)(lightlevel_full >> 0);
  if (condition == 0)     data[3] = 2;    // less than
  else            data[3] = 1;    // more than
  return write_to_module(MODULE_LIGHTSENSOR, (LIGHTSENSOR1_ADD + module_index), module_index, data, 5);
}

bool set_motion_trigger(uint8_t module_index, uint8_t condition)
{
  uint8_t data[3];
  data[0] = 0x10;
  if (condition == 0)     data[1] = 5;    // motion
  else if (condition == 1)  data[1] = 4;    // no motion
  return write_to_module(MODULE_PIR, (PIR1_ADD + module_index), module_index, data, 3);
}

bool set_accelerometer_trigger(uint8_t module_index, uint8_t condition)
{
  uint8_t data[3];
  data[0] = 0x11;
  if    (condition == 0)  data[1] = 1;    // put down
  else if (condition == 1)  data[1] = 0;    // pick up
  else            data[1] = 2;    // movement/tilt in any direction
  return write_to_module(MODULE_ACCELEROMETER, (ACCELEROMETER1_ADD + module_index), module_index, data, 3);
}

bool set_linetracker_trigger(uint8_t module_index, uint8_t condition)
{
  uint8_t data[3];
  data[0] = 0x15;
  if (condition == 1) data[1] = 2;        // no object in front of the sensor
  else        data[1] = 1;        // object in front of the sensor
  return write_to_module(MODULE_LINETRACKER, (LINETRACKER1_ADD + module_index), module_index, data, 3);
}

/*
void reset_module_i2c(uint8_t module_type, uint8_t address, uint8_t module_index, uint8_t *data)
{
  uint8_t modules_enum = modules_type_and_index_to_enum(module_type, module_index);
  if (check_if_module_attached(modules_enum))
  {
    i2c_write_module(address, data, sizeof(data));
  }
  else printf("Can`t send I2C-command, address %d not attached\n", address);
}
*/

bool reset_claw_trigger(uint8_t module_index)
{
  uint8_t data[3];
  data[0] = 0x15;
  data[1] = 0x00;
  return write_to_module(MODULE_CLAW, (CLAW1_ADD + module_index), module_index, data, 3);
}

bool reset_linetracker_trigger(uint8_t module_index)
{
  uint8_t data[3];
  data[0] = 0x16;
  data[1] = 0x00;
  return write_to_module(MODULE_LINETRACKER, (LINETRACKER1_ADD + module_index), module_index, data, 3);
}

bool reset_motor_encoder(uint8_t module_index)
{
  uint8_t data[2];
  data[0] = 0x12;
  data[1] = 0x00;
  return write_to_module(MODULE_MOTOR, (MOTOR1_ADD + module_index), module_index, data, 2);
}

bool reset_system(uint8_t module_index)
{
  return true;
}

bool reset_mod(uint8_t module_index)
{
  return true;
}

bool reset_motor(uint8_t module_index)
{
  set_motor_pwm((module_index), 0);
  set_drive_action_status(0);
  return true;
}

bool reset_rgb(uint8_t module_index)
{
  uint8_t data[2];
  data[0] = 0x13;
  data[1] = 0x00;
  return write_to_module(MODULE_RGB, (RGB1_ADD + module_index), module_index, data, 2);
}

bool reset_claw(uint8_t module_index)
{
  lock_claw(module_index);
  return reset_claw_trigger(module_index);
}

bool reset_servo(uint8_t module_index)
{
  //nothing
  return true;
}

bool reset_servo_proper(uint8_t module_index)
{
  //nothing
  return true;
}

bool reset_matrix(uint8_t module_index)
{
  set_matrix_action((module_index), 0, 0, 0);
  set_matrix_leds((module_index), 0);
  return true;
}

bool reset_button(uint8_t module_index)
{
  return set_button_trigger((module_index), 0);
}

bool reset_pir(uint8_t module_index)
{
  return set_motion_trigger((module_index), 0);
}

bool reset_accelerometer(uint8_t module_index)
{
  uint8_t data[2];
  data[0] = 0x10;
  return write_to_module(MODULE_ACCELEROMETER, (ACCELEROMETER1_ADD + module_index), module_index, data, 2);
}

bool reset_ultrasonic(uint8_t module_index)
{
  set_soundlevel_trigger((module_index), 0, 0);
  return set_distance_trigger((module_index), 0, 0);
}

bool reset_lightsensor(uint8_t module_index)
{
  return set_light_trigger((module_index), 0, 0);
}

bool reset_linetracker(uint8_t module_index)
{
  return reset_linetracker_trigger(module_index);
}

bool reset_display(uint8_t module_index)
{
  uint8_t data[2];
  data[0] = 0x16;
  data[1] = 0x00;
  return write_to_module(MODULE_DISPLAY, (DISPLAY1_ADD + module_index), module_index, data, 2);
}

bool reset_hinge(uint8_t module_index)
{
  return lock_hinge(module_index);
}

bool reset_knob(uint8_t module_index)
{
  uint8_t data[3];
  data[0] = 0x11;
  data[1] = 0x00;
  return write_to_module(MODULE_KNOB, (KNOB1_ADD + module_index), module_index, data, 3);
}

bool reset_weather(uint8_t module_index)
{
  return true;
}

bool check_mod(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  return true;
}

bool check_system(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  return true;
}

bool check_motors(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[14] = {0};
  i2c_read_module_data(address, data_rd, 14);

  if ((data_rd[7] == 0) && (data_rd[8] == 0) && (data_rd[9] == 0) && (data_rd[10] == 0) && (data_rd[11] == 0)) 
  { 
    //printf("Motor %02X completed action %d\n", address, action_id1);
    reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
    reset_drive_motors(action_id1);
    return true;
  }
  else return false;
}

bool check_servos(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[7] = {0};
  i2c_read_module_data(address, data_rd, 7);
  if (data_rd[6] == 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  else return false;
}

bool check_rgbs(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[6] = {0};
  i2c_read_module_data(address, data_rd, 6);
  if (data_rd[5] == 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  else return false;
}

bool check_matrices(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[22] = {0};
  i2c_read_module_data(address, data_rd, 21);
  if (data_rd[20] == 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  else return false;
}

bool check_buttons(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[3] = {0};
  i2c_read_module_data(address, data_rd, 3);
  if (data_rd[2] != 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  else return false;
}

bool check_pirs(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[8] = {0};
  i2c_read_module_data(address, data_rd, 8);
  if (data_rd[7] != 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  else return false; 
}

bool check_accelerometers(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[18] = {0};
  i2c_read_module_data(address, data_rd, 18);
  if ((data_rd[15] != 0) || (data_rd[16] != 0) || (data_rd[17] != 0)) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
}

bool check_ultrasonics(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[12] = {0};
  i2c_read_module_data(address, data_rd, 12);
  if (action_id1 != ACTION_NOT_SET)
  {
    if (data_rd[10] != 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  }
  if (action_id2 != ACTION_NOT_SET)
  {
    if (data_rd[11] != 0) return reset_modules_action_or_trigger(enumm, MODULES_SECOND_ACTION_OR_TRIGGER);
  }
  return false;
}

bool check_lightsensors(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[5] = {0};
  i2c_read_module_data(address, data_rd, 5);
  if (data_rd[3] != 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  else return false;
} 

bool check_linetrackers(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[18] = {0};
  i2c_read_module_data(address, data_rd, 18);
  if (action_id1 != ACTION_NOT_SET)
  {
    if ((data_rd[13] != 0 ) || (data_rd[14] != 0 ) || (data_rd[15] != 0 )) 
      return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  }
  /*
  if (action_id2 != ACTION_NOT_SET)
  {
    if (get_linetracker_action_status() == 0)
      reset_modules_action_or_trigger(enumm, MODULES_SECOND_ACTION_OR_TRIGGER);
  }
  */
  return false;
}

bool check_servo2s(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  //none yet
  return true;
}

bool check_displays(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[3] = {0};
  i2c_read_module_data(address, data_rd, 3);
  if (data_rd[1] == 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  else return false;
}

bool check_hinges(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[12] = {0};
  i2c_read_module_data(address, data_rd, 12);
  if (data_rd[7] == 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  else return false;
}

bool check_claws(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[12] = {0};
  i2c_read_module_data(address, data_rd, 12);
  if (action_id1 != ACTION_NOT_SET)
  {
    if (data_rd[10] == 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  }
  if (action_id2 != ACTION_NOT_SET)
  {
    if (data_rd[9] != 0) return reset_modules_action_or_trigger(enumm, MODULES_SECOND_ACTION_OR_TRIGGER);
  }
  return false;
}

bool check_knobs(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
  uint8_t data_rd[12] = {0};
  i2c_read_module_data(address, data_rd, 12);
  if (data_rd[4] == 0) return reset_modules_action_or_trigger(enumm, MODULES_FIRST_ACTION_OR_TRIGGER);
  else return false;
}

bool check_weather(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2)
{
 return false;
}
