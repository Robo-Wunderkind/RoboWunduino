#ifndef MAIN_MODULES_H
#define MAIN_MODULES_H

#include "Arduino.h"
#include "module_functions.h"
#include "module_handler.h"

/*----------------------------------------------------------- System -----------------------------------------------------------------*/

class _System
{
  public:
    _System();
    void set_leds(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3, uint8_t pwm4);
    void battery_level();
    void play_audio(uint8_t clip_id);
    
  private:
  
   
};

/*========================================================= ACTUATORS =======================================================================================*/

/*----------------------------------------------------------- Motor -----------------------------------------------------------------*/
#define ENABLE                            1
#define DISABLE                           0 
#define RIGHT                             1
#define LEFT                              0 

class _Motor
{
  public:
    _Motor();
    bool stop(uint8_t module_num);
    bool stop_all();
    bool torque(uint8_t module_num, int8_t torque_pct);
    bool speed(uint8_t module_num, int8_t speed_pct);
    bool drive(uint8_t module_index, int8_t speed, uint16_t distance, uint16_t wheeldiameter = WHEEL_DIAMETER);
    bool turn(int8_t speed_pct, float angle, uint16_t turning_diameter = DEFAULT_TURNING_DIAMETER_CM, uint16_t wheeldiameter = WHEEL_DIAMETER);
    bool backward(int8_t speed_pct, uint16_t distance, uint16_t wheeldiameter = WHEEL_DIAMETER);
    bool forward(int8_t speed_pct, uint16_t distance, uint16_t wheeldiameter = WHEEL_DIAMETER);
    void config(bool *motors, bool *directions);
    
  private:
    uint8_t _config_bitmask      = 0x03;
    uint8_t _directions_bitmask  = 0x02; // 0x01 or 0x02
    bool _mtr_cfg[MAX_MOTORS] = {ENABLE,ENABLE,DISABLE,DISABLE,DISABLE,DISABLE};            
    bool _dir_cfg[MAX_MOTORS] = {RIGHT,LEFT,LEFT,LEFT,LEFT,LEFT};  
    
    bool set_drives(int8_t speed_pct_L, int8_t speed_pct_R, uint16_t distance_L, uint16_t distance_R, uint16_t wheelD = WHEEL_DIAMETER);   
};

/*----------------------------------------------------------- Servo -----------------------------------------------------------------*/

class _ServoMotor
{
  public:
    _ServoMotor();
    bool set_position(uint8_t id, uint8_t pwm);
    uint16_t read_encoder(uint8_t id);
    
  private:
    
};

/*----------------------------------------------------------- Claw -----------------------------------------------------------------*/

class _Claw
{
  public:
    _Claw();
    bool set_position(uint8_t id, uint8_t pwm);
    uint16_t read_encoder(uint8_t id);
    
  private:
    
};

/*----------------------------------------------------------- Hinge -----------------------------------------------------------------*/

class _Hinge
{
  public:
    _Hinge();
    bool set_position(uint8_t id, uint8_t pwm);
    uint16_t read_encoder(uint8_t id);
    
  private:
    
};

/*----------------------------------------------------------- Display -----------------------------------------------------------------*/

#define ROBO_LOGO {0x00, 0x00, 0xff, 0xfc, 0x80, 0x02, 0xf0, 0x01, 0xcc, 0x39, 0x86, 0x45, 0x82, 0x45, 0xe6, 0x39, 0xbc, 0x02, 0xff, 0xfc, 0xfe, 0x3f, 0xfe, 0x41, 0xfe, 0x41, 0xfe, 0x41, 0xfe, 0x41, 0xfe, 0x3f}
#define X {0x80, 0x03, 0x40, 0x04, 0x20, 0x08, 0x10, 0x08, 0x08, 0x10, 0x0c, 0x20, 0x02, 0x20, 0x01, 0x40, 0x01, 0x80, 0x01, 0x80, 0x02, 0x40, 0x00, 0x60, 0x0c, 0x20, 0x10, 0x10, 0x60, 0x08, 0xc0, 0x06}
#define LOGO {0xff, 0Xf8, 0x80, 0x04, 0x80, 0x022, 0xb0, 0x1a, 0xb3, 0x9a, 0x83, 0x82, 0x80, 0x04, 0xff, 0xf8, 0x00, 0x00, 0xfe, 0x38, 0x82, 0x44, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x44, 0xfe, 0x38}

class _Display
{
  public:
    _Display();
    bool text(uint8_t module_index, uint8_t orientation, char* text, uint16_t scrolling_rate_ms = TEXT_RATE_DEFAULT);
    bool animation(uint8_t module_index, uint8_t orientation, uint8_t animation_num, uint16_t frame_rate = FRAME_RATE_DEFAULT);
    //bool image(uint8_t module_index, const uint8_t* image, uint8_t orientation, uint16_t time_ms);
    bool custom_image(uint8_t module_index, const uint8_t* image, uint8_t orientation, uint16_t time_ms);
    bool custom_animation(uint8_t module_index, const uint8_t (*frames)[32], uint8_t num_frames, uint8_t orientation, uint16_t frame_rate = FRAME_RATE_DEFAULT);

    const uint8_t test_image1[32] = ROBO_LOGO;
    const uint8_t test_image2[32] = X;
    const uint8_t test_animation[3][32] = {LOGO, X, LOGO};
    const uint8_t test_animation_size = sizeof(this->test_animation) / sizeof(this->test_animation[0]); 
  private:
    
};

/*----------------------------------------------------------- LED -----------------------------------------------------------------*/

class _LED
{
  public:
    _LED();
    bool rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t module_num);
    bool blink(uint8_t r, uint8_t g, uint8_t b, uint8_t blinks, float frequency, uint8_t module_num);
    
  private:
    
};

/*========================================================= SENSORS =======================================================================================*/

/*-------------------------------------------------- Ultrasonic Distance Sensor ---------------------------------------------------------------------------*/

class _Ultrasonic
{
  public:
    _Ultrasonic();
    uint16_t read_ultrasonic(uint8_t module_num);
    
  private:
 
};

/*----------------------------------------------------------- IMU -----------------------------------------------------------------*/

class _IMU
{
  public:
    _IMU();
    bool read_gyroscope(uint8_t id, float *gyrox, float *gyroy, float *gyroz);
    bool read_accelerometer(uint8_t id, float *accx, float *accy, float *accz);
    
  private:
    
};

/*----------------------------------------------------------- Button -----------------------------------------------------------------*/

class _Button
{
  public:
    _Button();
    uint8_t read(uint8_t id);
    
  private:
    
};

/*-------------------------------------------------------- Line Tracker -----------------------------------------------------------------*/

class _LineTracker
{
  public:
    _LineTracker();
    bool read_all(uint8_t module_index, linetracker_sensorvalues *values);
    bool read_left(uint8_t module_num);
    bool read_center(uint8_t module_num);
    bool read_right(uint8_t module_num);
    uint16_t read_left_prox(uint8_t module_num);
    uint16_t read_center_prox(uint8_t module_num);
    uint16_t read_right_prox(uint8_t module_num);
    
  private:
    
};

/*-------------------------------------------------------- Light Sensor -----------------------------------------------------------------*/

class _Light
{
  public:
    _Light();
    uint16_t read(uint8_t module_num);
    
  private:
    
};

/*-------------------------------------------------------- Motion Sensor -----------------------------------------------------------------*/

class _Motion
{
  public:
    _Motion();
    uint8_t read(uint8_t module_num);
    
  private:
    
};

/*------------------------------------------------------------ Knob -----------------------------------------------------------------*/

class _Knob
{
  public:
    _Knob();
    uint16_t read(uint8_t id);
    
  private:
    
};

/*
class Lidar
{
  public:
    
  private:
    
};

class Weathersensor
{
  public:
    
  private:
    
};
*/













#endif
