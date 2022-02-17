#ifndef MAIN_MODULES_H
#define MAIN_MODULES_H

#include "Arduino.h"
#include "module_functions.h"
#include "module_handler.h"

void scan();

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

/*========================================================= ACTUATORS ==================================================================================*/

/*----------------------------------------------------------- Motor ------------------------------------------------------------------------------------*/
#define ENABLE                            1
#define DISABLE                           0 
#define RIGHT                             1
#define LEFT                              0 

class _Motor
{
  public:
    _Motor();

     /*
     *  Stops one motor in particular 
     *
     *  Parameters:
     *    - uint8_t module_index: which motor cube? (0-5)
     *
     *  Return value: 
     *    - bool result: indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool stop(uint8_t module_index);

     /*
     *  Stops all motors 
     *
     *  Parameters:
     *
     *  Return value: 
     *    - bool result: indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool stop_all();

     /*
     *  Sets the torque of the motor
     *
     *  Parameters:
     *    - uint8_t module_index: which motor cube? (0-5)
     *    - int8_t torque_pct: -100% -> 100% torque
     *
     *  Return value: 
     *    - bool result: indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool torque(uint8_t module_index, int8_t torque_pct);

     /*
     *  Sets the speed of the motor, the motor will try to maintain its velocity once set
     *
     *  Parameters:
     *    - uint8_t module_index: which motor cube? (0-5)
     *    - int8_t speed_pct: -100% -> 100% velocity
     *   
     *  Return value: 
     *    - bool result: indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool speed(uint8_t module_index, int8_t speed_pct);

     /*
     *  Drives a single motor to rotate a specified distance at a specified velocity
     *
     *  Parameters:
     *    - uint8_t module_index: which motor cube? (0-5)
     *    - int8_t speed: -100% -> 100% velocity
     *    - uint16_t distance: distance in cm to drive
     *    - uint16_t wheeldiameter: optional, if different wheels are attached specify the diameter in mm so we can know how many motor turns are needed to drive the distance
     *
     *  Return value: 
     *    - bool result: indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool drive(uint8_t module_index, int8_t speed, uint16_t distance, uint16_t wheeldiameter = WHEEL_DIAMETER);

     /*
     *  Turns Robo 
     *
     *  Parameters:
     *    - int8_t speed_pct: -100% -> 100% velocity
     *    - float angle: angle to turn
     *    - uint16_t turning_diameter: optional, the distance from wheel to wheel, needed to know what distance is needed to travel to achieve the angle
     *    - uint16_t wheeldiameter: optional, if different wheels are attached specify the diameter in mm so we can know how many motor turns are needed to drive the distance
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool turn(int8_t speed_pct, float angle, uint16_t turning_diameter = DEFAULT_TURNING_DIAMETER_CM, uint16_t wheeldiameter = WHEEL_DIAMETER);

     /*
     *  Drives Robo Backwards
     *
     *  Parameters:
     *    - int8_t speed_pct: -100% -> 100% velocity
     *    - uint16_t distance: distance in cm to drive
     *    - uint16_t wheeldiameter: optional, if different wheels are attached specify the diameter in mm so we can know how many motor turns are needed to drive the distance
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool backward(int8_t speed_pct, uint16_t distance, uint16_t wheeldiameter = WHEEL_DIAMETER);

     /*
     *  Drives Robo Forward 
     *
     *  Parameters:
     *    - int8_t speed_pct: -100% -> 100% velocity
     *    - uint16_t distance: distance in cm to drive
     *    - uint16_t wheeldiameter: optional, if different wheels are attached specify the diameter in mm so we can know how many motor turns are needed to drive the distance
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool forward(int8_t speed_pct, uint16_t distance, uint16_t wheeldiameter = WHEEL_DIAMETER);

     /*
     *  Sets the motor configuration for Robo 
     *
     *  Parameters:
     *    - bool *motors: Array of booleans to show which motors are enabled [0->5] ENABLE/DISABLE
     *    - bool *directions: Array of booleans to show which direction motors are turning [0->5] LEFT/RIGHT
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    void config(bool *motors, bool *directions);
    
  private:
    uint8_t _config_bitmask      = 0x03;
    uint8_t _directions_bitmask  = 0x02; // 0x01 or 0x02
    bool _mtr_cfg[MAX_MOTORS] = {ENABLE,ENABLE,DISABLE,DISABLE,DISABLE,DISABLE};            
    bool _dir_cfg[MAX_MOTORS] = {RIGHT,LEFT,LEFT,LEFT,LEFT,LEFT};  

     /*
     *  Helper function to set the drive command
     *
     *  Parameters:
     *    - int8_t speed_pct_L: speed for left side motors
     *    - int8_t speed_pct_R: speed for right side motors
     *    - uint16_t distance_L: distance to travel for motors on the left side
     *    - uint16_t distance_R: distance to travel for motors on the right side
     *    - uint16_t wheelD: optional, if different wheels are attached specify the diameter in mm so we can know how many motor turns are needed to drive the distance
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool set_drives(int8_t speed_pct_L, int8_t speed_pct_R, uint16_t distance_L, uint16_t distance_R, uint16_t wheelD = WHEEL_DIAMETER);   
};

/*----------------------------------------------------------- Servo ---------------------------------------------------------------------------*/

class _ServoMotor
{
  public:
    _ServoMotor();

     /*
     *  Sets the angular position of the Servo
     *
     *  Parameters:
     *    - uint8_t module_index: which servo cube? (0-3)
     *    - int16_t angle: the angular position of the servo
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool set_position(uint8_t module_index, int16_t angle);

     /*
     *  Gets the current encoder reading
     *
     *  Parameters:
     *    - uint8_t module_index: which servo cube? (0-3)
     *
     *  Return value: 
     *    - uint16_t encoder: the encoder reading 0-1023
     */
    uint16_t read_encoder(uint8_t module_index);
    
};

/*----------------------------------------------------------- Claw ----------------------------------------------------------------------------*/

class _Claw
{
  public:
    _Claw();

     /*
     *  Opens the claw
     *
     *  Parameters:
     *    - uint8_t module_index: which claw? (0-3)
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool open(uint8_t module_index);

    /*
     *  Closes the claw
     *
     *  Parameters:
     *    - uint8_t module_index: which claw? (0-3)
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool close(uint8_t module_index);

     /*
     *  Tells you if the claw is opened or closed
     *
     *  Parameters:
     *    - uint8_t module_index: which claw? (0-3)
     *
     *  Return value: 
     *    - bool, indicates if whether the claw is opened (1) or Closed (0) or Unknown(-1)
     */
    int8_t get_state(uint8_t module_index);

     /*
     *  Reports the proximity sensor value
     *
     *  Parameters:
     *    - uint8_t module_index: which claw? (0-3)
     *
     *  Return value: 
     *    - uint16_t value: IR Intensity, higher when there is an object and lower when there is no object
     */
    int16_t get_proximity(uint8_t module_index);
    
};

/*----------------------------------------------------------- Hinge --------------------------------------------------------------------------*/

class _Hinge
{
  public:
    _Hinge();

     /*
     *  Sets the angular position of the Hinge
     *
     *  Parameters:
     *    - uint8_t module_index: which Hinge? (0-3)
     *    - int16_t angle: the angular position of the hinge
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool set_position(uint8_t module_index, int16_t angle);

     /*
     *  Gets the current encoder reading
     *
     *  Parameters:
     *    - uint8_t module_index: which hinge? (0-3)
     *
     *  Return value: 
     *    - uint16_t encoder: the encoder reading 0-1023
     */
    uint16_t read_encoder(uint8_t module_index);
    
};

/*----------------------------------------------------------- Display ------------------------------------------------------------------------*/

#define ROBO_LOGO {0x00, 0x00, 0x7f, 0xf8, 0x7f, 0xfc, 0x67, 0xe6, 0x67, 0xe6, 0x7f, 0xfe, 0x7c, 0x3e, 0x7e, 0x7c, 0x7f, 0xf8, 0x00, 0x00, 0x7e, 0x3c, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x3c}
#define CHECK_MARK {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x07, 0x00, 0x0e, 0x00, 0x1c, 0x00, 0x38, 0x00, 0x70, 0x00, 0xe0, 0xc1, 0xc0, 0xe3, 0x80, 0x77, 0x00, 0x3e, 0x00, 0x1c, 0x00}
#define STOP {0x07, 0xe0, 0x08, 0x10, 0x10, 0x08, 0x20, 0x04, 0x40, 0x02, 0xbf, 0xef, 0xa2, 0xab, 0xba, 0xaf, 0x8a, 0xa9, 0xba, 0xe9, 0x80, 0x01, 0x40, 0x02, 0x20, 0x04, 0x10, 0x08, 0x08, 0x10, 0x07, 0xe0}
#define GO {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x7e, 0xfe, 0xff, 0xc0, 0xc3, 0xc0, 0xc3, 0xde, 0xc3, 0xde, 0xc3, 0xc6, 0xc3, 0xfe, 0xff, 0x7c, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define UP {0x01, 0x80, 0x03, 0xc0, 0x07, 0xe0, 0x0f, 0xf0, 0x1f, 0xf8, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

class _Display
{
  public:
    _Display();
    
    /*
     *  Scrolls any ascii text across the display cube 
     *
     *  Parameters:
     *    - uint8_t module_index: which display cube? (0-3)
     *    - uint8_t orientation: the orientation of the text 0, 90, 180 and 270 degrees
     *    - char* text: array of text to display
     *    - uint16_t scrolling_rate_ms: optional parameter to change the speed of the text scrolling across the screen in milliseconds per column shift
     *
     *  Return value: bool, indicates if the command was sent properly, if not we automatically scan for modules
     *    -
     */
    bool text(uint8_t module_index, uint8_t orientation, char* text, uint16_t scrolling_rate_ms = TEXT_RATE_DEFAULT);
    
     /*
     *  Scrolls any ascii text across the display cube 
     *
     *  Parameters:
     *    - uint8_t module_index: which display cube? (0-3)
     *    - uint8_t orientation: the orientation of the text 0, 90, 180 and 270 degrees
     *    - char* text: array of text to display
     *    - uint16_t scrolling_rate_ms: optional parameter to change the speed of the text scrolling across the screen in milliseconds per column shift
     *
     *  Return value: bool, indicates if the command was sent properly, if not we automatically scan for modules
     *    -
     */
    bool animation(uint8_t module_index, uint8_t orientation, uint8_t animation_num, uint16_t frame_rate = FRAME_RATE_DEFAULT);

     /*
     *  Scrolls any ascii text across the display cube 
     *
     *  Parameters:
     *    - uint8_t module_index: which display cube? (0-3)
     *    - uint8_t orientation: the orientation of the text 0, 90, 180 and 270 degrees
     *    - char* text: array of text to display
     *    - uint16_t scrolling_rate_ms: optional parameter to change the speed of the text scrolling across the screen in milliseconds per column shift
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool custom_image(uint8_t module_index, const uint8_t* image, uint8_t orientation, uint16_t time_ms);

     /*
     *  Scrolls any ascii text across the display cube 
     *
     *  Parameters:
     *    - uint8_t module_index: which display cube? (0-3)
     *    - uint8_t orientation: the orientation of the text 0, 90, 180 and 270 degrees
     *    - char* text: array of text to display
     *    - uint16_t scrolling_rate_ms: optional parameter to change the speed of the text scrolling across the screen in milliseconds per column shift
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool custom_animation(uint8_t module_index, const uint8_t (*frames)[32], uint8_t num_frames, uint8_t orientation, uint16_t frame_rate = FRAME_RATE_DEFAULT);
    
};

/*----------------------------------------------------------- LED --------------------------------------------------------------------------*/


class _LED
{
  public:
    _LED();

     /*
     *  Sets the LED to the specified RGB value
     *
     *  Parameters:
     *    - uint8_t module_index: which LED? (0-7)
     *    - uint8_t r: red component
     *    - uint8_t g: green component
     *    - uint8_t b: blue component
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool rgb(uint8_t module_index, uint8_t r, uint8_t g, uint8_t b);

     /*
     *  Blinks the led in the desired colour for the specified number of blinks at the given frequency 
     *
     *  Parameters:
     *    - uint8_t module_index: which LED? (0-7)
     *    - uint8_t r: red component
     *    - uint8_t g: green component
     *    - uint8_t b: blue component
     *    - uint8_t blinks: number of blinks
     *    - float frequency: how fast is the blinking? In Hz
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool blink(uint8_t module_index, uint8_t r, uint8_t g, uint8_t b, uint8_t blinks, float frequency);
};

/*========================================================= SENSORS ======================================================================*/

/*-------------------------------------------------- Ultrasonic Distance Sensor ----------------------------------------------------------*/

class _Ultrasonic
{
  public:
    _Ultrasonic();

     /*
     *  Reads the distance measurement in centimeters 
     *
     *  Parameters:
     *    - uint8_t module_index: which ultrasonic cube? (0-3)
     *
     *  Return value: 
     *    - uint16_t distance: distance measurement in centimeters
     */
    uint16_t read(uint8_t module_index);     
};

/*----------------------------------------------------------- IMU ------------------------------------------------------------------------*/

class _IMU
{
  public:
    _IMU();

     /*
     *  Fetches the x, y and z components of the gyroscope reading
     *
     *  Parameters:
     *    - uint8_t module_index: which imu cube? (0-3)
     *    - float *gyrox: pointer to your x axis gyroscope variable
     *    - float *gyroy: pointer to your y axis gyroscope variable
     *    - float *gyroz: pointer to your z axis gyroscope variable
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool read_gyroscope(uint8_t module_index, float *gyrox, float *gyroy, float *gyroz);

     /*
     *  Fetches the x, y and z components of the accelerometer reading
     *
     *  Parameters:
     *    - uint8_t module_index: which imu cube? (0-3)
     *    - float *accx: pointer to your x axis accelerometer variable
     *    - float *accy: pointer to your y axis accelerometer variable
     *    - float *accz: pointer to your z axis accelerometer variable
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool read_accelerometer(uint8_t module_index, float *accx, float *accy, float *accz);
};

/*----------------------------------------------------------- Button --------------------------------------------------------------------*/

class _Button
{
  public:
    _Button();

     /*
     *  Returns the state of the button
     *
     *  Parameters:
     *    - uint8_t module_index: which button? (0-3)
     *
     *  Return value: 
     *    - uint8_t button_state: the state of the button
     */
    uint8_t read(uint8_t module_index);
};

/*-------------------------------------------------------- Line Tracker -----------------------------------------------------------------*/

class _LineTracker
{
  public:
    _LineTracker();

     /*
     *  Reads and fills the complete linetracker_values structure 
     *
     *  Parameters:
     *    - uint8_t module_index: which linetracker? (0-3)
     *    - linetracker_sensorvalues *values: pointer to linetracker_sensorvalues structure
     *
     *  Return value: 
     *    - bool, indicates if the command was sent properly, if not we automatically scan for modules
     */
    bool read_all(uint8_t module_index, linetracker_sensorvalues *values);

     /*
     *  Returns a ture or false indicating whether or not there is an object/white surface detected on the left sensor
     *
     *  Parameters:
     *    - uint8_t module_index: which linetracker? (0-3)
     *  
     *  Return value: 
     *    - bool, indicates if there is an object/white surface detected on the left sensor
     */
    bool read_left(uint8_t module_index);

     /*
     *  Returns a true or false indicating whether or not there is an object/white surface detected on the center sensor
     *
     *  Parameters:
     *    - uint8_t module_index: which linetracker? (0-3)
     *  
     *  Return value: 
     *    - bool, indicates if there is an object/white surface detected on the center sensor
     */
    bool read_center(uint8_t module_index);

     /*
     *  Returns a true or false indicating whether or not there is an object/white surface detected on the right sensor
     *
     *  Parameters:
     *    - uint8_t module_index: which linetracker? (0-3)
     *  
     *  Return value: 
     *    - bool, indicates if there is an object/white surface detected on the right sensor
     */
    bool read_right(uint8_t module_index);

     /*
     *  Returns an analog signal of the left proximity sensor
     *
     *  Parameters:
     *    - uint8_t module_index: which linetracker? (0-3)
     *  
     *  Return value: 
     *    - uint16_t prox: indicates the amount of IR light received from the emitter; useful to know if an object or white surface has been detected
     */
    uint16_t read_left_prox(uint8_t module_index);

     /*
     *  Returns an analog signal of the center proximity sensor
     *
     *  Parameters:
     *    - uint8_t module_index: which linetracker? (0-3)
     *  
     *  Return value: 
     *    - uint16_t prox: indicates the amount of IR light received from the emitter; useful to know if an object or white surface has been detected
     */
    uint16_t read_center_prox(uint8_t module_index);

     /*
     *  Returns an analog signal of the right proximity sensor
     *
     *  Parameters:
     *    - uint8_t module_index: which linetracker? (0-3)
     *  
     *  Return value: 
     *    - uint16_t prox: indicates the amount of IR light received from the emitter; useful to know if an object or white surface has been detected
     */
    uint16_t read_right_prox(uint8_t module_index);
};

/*-------------------------------------------------------- Light Sensor -----------------------------------------------------------------*/

class _Light
{
  public:
    _Light();

     /*
     *  Returns the intensity of light in LUX from the light sensor
     *
     *  Parameters:
     *    - uint8_t module_index: which light sensor? (0-3)
     *  
     *  Return value: 
     *    - uint16_t prox: indicates the amount of visible light received in LUX
     */
    uint16_t read(uint8_t module_index);
};

/*-------------------------------------------------------- Motion Sensor -----------------------------------------------------------------*/

class _Motion
{
  public:
    _Motion();

     /*
     *  Returns the state of the motion sensor
     *
     *  Parameters:
     *    - uint8_t module_index: which motion sensor? (0-3)
     *
     *  Return value: 
     *    - uint8_t motion_state: whther or not there is motion detected
     */
    uint8_t read(uint8_t module_index);
};

/*------------------------------------------------------------ Knob -----------------------------------------------------------------*/

class _Knob
{
  public:
    _Knob();

     /*
     *  Returns an analog signal of the knob
     *
     *  Parameters:
     *    - uint8_t module_index: which knob? (0-3)
     *  
     *  Return value: 
     *    - uint16_t position: returns an analog reading of the knob position 0-1023 0-> 180 degrees
     */
    uint16_t read(uint8_t module_index);
};

/*------------------------------------------------------------ Weather -----------------------------------------------------------------*/
// Need to isolate the sensors much better to remove them from any heat
class _Weather
{
  public:
    _Weather();

    uint16_t read_tvoc(uint8_t module_index);
    uint16_t read_ethanol(uint8_t module_index);
    uint16_t read_h2(uint8_t module_index);
    uint16_t read_eco2(uint8_t module_index);
    float    read_temp_c(uint8_t module_index);
    float    read_temp_f(uint8_t module_index);
    float    read_pressure_mb(uint8_t module_index);
    float    read_pressure_kpa(uint8_t module_index);
    float    read_humidity_rh(uint8_t module_index);
    float    read_analog_temp(uint8_t module_index);
    float    read_altitude_m(uint8_t module_index, float sealevel_mb);
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
