/*
 * modules.h
 *
 */

#ifndef MAIN_MODULES_H_
#define MAIN_MODULES_H_

#include "Arduino.h"
// CONVERSION RATES CONFIG:

#define LINETRACKER_SATURATION                2500
#define ULTRASONIC_DISTANCE_CONVERSION_RATE 	0.0084
#define SOUNDLEVEL_CONVERSION_RATE 				    0.8
#define LIGHTLEVEL_CONVERSION_RATE				    0.8
#define MOTOR_SPEED_CONVERSION_RATE				    2.34//2.6
#define SERVO_ANGLE_CONVERSION_RATE				    1
#define SERVO_ANGLE_RANGE_DEGREES				      360
#define ACC_4G_CONV_G                         8192.0
#define GYROSCOPE_SENSITIVITY                 131.0
#define DEFAULT_TURNING_DIAMETER_CM           16.5
#define WHEEL_DIAMETER                        0x5A
#define MAX_DISTANCE                          0xFFF0
#define TEXT_RATE_DEFAULT                     75 
#define FRAME_RATE_DEFAULT                    300 
#define PI                                    3.14159265359
#define MAXVEL                                1000
#define MINVEL                                15

typedef struct
{
  uint8_t bin_l;
  uint8_t bin_c;
  uint8_t bin_r;
  uint8_t edge_l;
  uint8_t edge_c;
  uint8_t edge_r;
  uint16_t l;
  uint16_t c;
  uint16_t r;
  uint8_t black_white;
} linetracker_sensorvalues;


/*
 *  Different module set and get functions.
 *
 *
 * 	First step is checking if the module is attached. If not, get functions return 0.
 *
 *  All set functions fill an array with module command and data and then use "i2c_write_module"
 *  to write into the i2c in buffer of the module.
 *
 *  All get functions use "i2c_read_module_data" to read the modules i2c out buffer,
 *  then convert the needed data and return it.
 *
 *  Parameters:
 *  	- uint8_t module_index: motor 1 has index 0, motor 2 has index 1, ...
 *  	- ...
 *
 *  Return value:
 *  	- ...
 */

/*
 *	Gets the current id set in the module's eeprom
 *
 *  Parameters:
 *  	- I2C address
 *
 *  Return value: eeprom ID
 *  	-
 */
uint8_t get_eeprom_module_id(uint8_t address);

/*
 *	Gets the current id set in the module's eeprom
 *
 *  Parameters:
 *  	- module type ex. MODULE_DISPLAY, ID to be set, I2C address
 *
 *  Return value: eeprom ID
 *  	-
 */
bool set_eeprom_module_id(uint8_t address, uint8_t id);


/*
 *	Sets the current servo position as '0'
 *
 *  Parameters:
 *  	- uint8_t module_index
 *
 *  Return value:
 *  	-
 */
bool set_zero_servo(uint8_t module_index);

/*
 *	Sets motor to turn the desitred angle
 *
 *  Parameters:
 *  	int16_t angle
 *
 *  Return value: true/false
 *  	-
 */
bool set_motor_angle(uint8_t module_index, int16_t angle, uint8_t direction);

bool lock_hinge(uint8_t module_index);
bool set_hinge_torque(uint8_t module_index, int8_t speed);
bool set_hinge_position(uint8_t module_index, int8_t angle);

bool lock_claw(uint8_t module_index);

/*
 *	Sets Claw Proximity Trigger
 *
 *  Parameters:
 *  	- int8_t condition: 0 = object, 1 = no object
 *
 *  Return value:
 *  	-
 */
bool set_claw_trigger(uint8_t module_index, int8_t condition);

/*
 *	Sets claw PWM
 *
 *  Parameters:
 *  	- int8_t speed: % (-100 - 0 - 100).
 *
 *  Return value:
 *  	-
 */
bool set_claw_torque(uint8_t module_index, int8_t speed);

/*
 *	Opens and Closes the Claw
 *
 *  Parameters:
 *  	- bool open close 1 = close, 0 = open
 *
 *  Return value:
 *  	-
 */
bool claw_open_close(uint8_t module_index, bool open_close);

/*
 *	Sets motor PWM, without speed control.
 *
 *  Parameters:
 *  	- uint8_t speed: % (-100 - 0 - 100).
 *
 *  Return value:
 *  	-
 */
bool set_motor_pwm(uint8_t module_index, int8_t speed);

/*
 *	Sets motor traveling speed, with speed control.
 *
 *  Parameters:
 *  	- uint8_t speedHSB/LSB: speed in mm/s (-100 - 0 - 100) * MOTOR_SPEED_CONVERSION_RATE.
 *  	- uint8_t wheel_diameterHSB/LSB: standard Robo wheel (gen 1 and 2) is 88mm.
 *
 *  Return value:
 *  	-
 */
bool set_motor_speed(uint8_t module_index, int8_t speed);
bool adjust_motor_speed(uint8_t module_index, uint8_t speedHSB, uint8_t speedLSB, uint8_t wheel_diameterHSB, uint8_t wheel_diameterLSB);
bool set_servo_position(uint8_t module_index, int8_t position);
bool set_rgb_color(uint8_t module_index, uint8_t r, uint8_t g, uint8_t b);
bool set_matrix_leds(uint8_t module_index, uint64_t leds);
bool reset_motor_encoder(uint8_t module_index);

// Display Functions
bool set_animation(uint8_t module_index, uint8_t animation_num, uint8_t repeats, uint8_t reverse, uint8_t orientation, uint8_t num_frames, uint8_t frame_rateH, uint8_t frame_rateL); 
bool set_display_image(uint8_t module_index, uint8_t image, uint8_t orientation, uint8_t delayH, uint8_t delayL);
bool load_custom_display_image(uint8_t module_index, uint8_t frame_half, uint8_t rows[]);
bool load_custom_display_animation(uint8_t module_index, uint8_t frame_index, uint8_t frame_half, uint8_t frame_rows[]);
bool load_custom_text(uint8_t module_index, char chars[], uint8_t starting_index, uint8_t length);
bool set_custom_display_text(uint8_t module_index, uint8_t orientation, uint8_t total_length, uint8_t scrolling_rateH, uint8_t scrolling_rateL);
bool reset_display(uint8_t module_index);

/*
 * 	Calibrates the linetracker. Module should be over a white surface while being calibrated.
 *
 *  Parameters:
 *  	-
 *
 *  Return value:
 *  	-
 */
bool calibrate_linetracker(uint8_t module_index);

bool read_claw_proximity(uint8_t module_index, uint8_t *prox_state);
bool read_ultrasonic_distance(uint8_t module_index, float *distance_cm);
bool read_ultrasonic_volume(uint8_t module_index, float *vol);
bool read_pir_state(uint8_t module_index, uint8_t *pir_state);
bool read_button_state(uint8_t module_index, uint8_t *button_state);
bool read_knob(uint8_t module_index, uint16_t *knob_reading);
bool read_lightsensor(uint8_t module_index, float *light);
bool read_motor_distance(uint8_t module_index, float *distance_in_cm);
bool read_linetracker_sensorvalues(uint8_t module_index, linetracker_sensorvalues *values, uint16_t sensor_saturation);
bool read_accelerometer_state(uint8_t module_index);
bool get_last_ir_cmd(uint8_t module_index);
bool reset_irblaster_trigger(uint8_t module_index);
bool read_gyro(uint8_t module_index, float *gyrox, float *gyroy, float *gyroz);
bool read_accelerometer_values(uint8_t module_index, float *accx, float *accy, float *accz);

/*
 * 	All set_action and set_trigger functions are used for Robo Code app.
 * 	Once an action is set, the module will fulfill it, for example servo moves to specific angle,
 * 	and then, once it is done, set the action status byte in the out buffer to "action done". By checking this byte the
 * 	system knows the action states of the different modules. Same for triggers, once trigger has occurred,
 * 	the module sets the related byte in the i2c out buffer. Setting an action or a trigger will reset the previous one,
 * 	if it was not done.
 */
bool set_motor_action(uint8_t module_index, int8_t speed, uint8_t wheel_diameterHSB, uint8_t wheel_diameterLSB, uint8_t distanceHSB, uint8_t distanceLSB);
bool set_servo_action(uint8_t module_index, uint8_t positionHSB, uint8_t positionLSB);
bool set_matrix_action(uint8_t module_index, uint64_t leds, uint8_t timeHSB, uint8_t timeLSB);
bool set_rgb_action(uint8_t module_index, uint8_t r, uint8_t g, uint8_t b, uint8_t timeHSB, uint8_t timeLSB, uint8_t blinks ,uint8_t pulse);

/*
 *  As the max payload length of a single BLE message is 17 byte, two messages are used for text scrolling.
 *  Function does only prepare data, does not send data to module, this is done by "matrix_set_string".
 *
 *  Parameters:
 *  	- uint8_t text_orientation: text orientation can be changed, 0 = 0°, 1 = 90°, 2 = 180°, 3 = 270°.
 *  	- uint8_t repeats: specifies how many times the same text should be scrolled, 22 = infinite.
 *  	- uint8_t scrolling_rate: scrolling speed
 *  	- uint8_t stringh_length: complete string length
 *  	- uint8_t *string: first portion of the string to scroll
 *
 *  Return value:
 *  	- ...
 */
bool set_matrix_string_action_0(uint8_t module_index, uint8_t text_orientation, uint8_t repeats, uint8_t scrolling_rate, uint8_t string_length, uint8_t *string);

/*
 *  If the string to scroll is bigger then 11 bytes, second portion is set by this function.
 *  Function does only prepare data, does not send data to module, this is done by "matrix_set_string".
 *
 *  Parameters:
 *  	- uint8_t stringh_length: complete string length
 *  	- uint8_t *string: second portion of the string to scroll
 *
 *  Return value:
 *  	- ...
 */
bool set_matrix_string_action_1(uint8_t string_length, uint8_t *string);

/*
 * 	Used by "set_matrix_string_action_0" and "set_matrix_string_action_1" to send stored string
 * 	to module. string already contains length, orientation, scrolling rate and repeats.
 *
 *  Parameters:
 *  	-
 *
 *  Return value:
 *  	-
 */
bool matrix_set_string();

bool set_distance_trigger(uint8_t module_index, uint8_t condition, uint8_t distance_in_cm);
bool set_button_trigger(uint8_t module_index, int8_t condition);
bool set_light_trigger(uint8_t module_index, uint8_t condition, uint16_t lightlevel);
bool set_motion_trigger(uint8_t module_index, uint8_t condition);
bool set_soundlevel_trigger(uint8_t module_index, uint8_t condition, uint8_t soundlevel);
bool set_accelerometer_trigger(uint8_t module_index, uint8_t condition);
bool set_linetracker_trigger(uint8_t module_index, uint8_t condition);
bool reset_linetracker_trigger(uint8_t module_index);
bool set_ir_trigger(uint8_t byte_ll, uint8_t byte_l, uint8_t byte_h, uint8_t byte_hh);

// Reset Functions
bool reset_system(uint8_t address);
bool reset_motor(uint8_t address);
bool reset_servo(uint8_t address);
bool reset_rgb(uint8_t module_index);
bool reset_matrix(uint8_t address);
bool reset_button(uint8_t address);
bool reset_pir(uint8_t address);
bool reset_accelerometer(uint8_t module_index);
bool reset_ultrasonic(uint8_t address);
bool reset_lightsensor(uint8_t address);
bool reset_linetracker(uint8_t address);
bool reset_display(uint8_t module_index);
bool reset_servo_proper(uint8_t module_index);
bool reset_claw(uint8_t module_index);
bool reset_hinge(uint8_t module_index);
bool reset_knob(uint8_t module_index);

bool reset_mod(uint8_t module_index);

// Check Functions
bool check_system(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_motors(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_servos(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_rgbs(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_matrices(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_buttons(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_pirs(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_accelerometers(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_ultrasonics(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_lightsensors(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_linetrackers(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_servo2s(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_displays(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_hinges(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_claws(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_mod(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);
bool check_knobs(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);


#endif /* MAIN_MODULES_H_ */
