/*
 * robowunderkind.h
 *
 */

#ifndef MAIN_ROBOWUNDERKIND_H_
#define MAIN_ROBOWUNDERKIND_H_

#include "Arduino.h"
#include "i2c_bus.h"
#include "modules.h"
#include "module_functions.h"

#define BATTERY_UNINITIALISED				      0
#define BATTERY_DISCHARGING					      1
#define BATTERY_CHARGING					        2
#define BATTERY_FULL     				  	      3

#define LOW_BATTERY_THRESHOLD			        25

#define USB_UNPLUGGED						          1
#define USB_PLUGGED							          2
#define USB_CHARGING_DONE                 3

#define GPIO_BOOST_IC_EN         	        5
#define GPIO_AUDIO_EN                     14
#define GPIO_BOOST_EN						          18
#define GPIO_ONOFF	         				      23
#define GPIO_CHARGE_SENSE         	      34
#define GPIO_USB_DET         				      35
#define GPIO_BATTERY_SENSE       		      39

#define LED1                              32 
#define LED2                              33
#define LED3                              26
#define LED4                              27

#define BOOST_CONVERTER_OFF					      0
#define BOOST_CONVERTER_ON					      1

#define OFF_TIME                          2
#define DEFAULT_VREF    				  	      1100
#define NO_OF_SAMPLES   					        64
#define BATTERY_CONVERSION_RATIO		      1.31
#define BATTERY_LEVEL_THRESHOLD_COUNT		  10
#define BATTERY_LEVEL_THRESHOLD_COUNT_OFF	60

#define BATTERY_LEVEL_SHUTDOWN				    10

void print_heap();
void scan();

class RoboWunderkind //: public LED , public Motor etc
{
  private:
    bool _boost_converter_state = false;
    uint8_t _mac_addr[6] = {0};

  public:
    _Motor Motor = _Motor();
    _ServoMotor ServoMotor = _ServoMotor();
    _Claw Claw = _Claw();
    _Hinge Hinge = _Hinge();
    _Display Display = _Display();
    _LED LED = _LED();
    _Ultrasonic Ultrasonic = _Ultrasonic();
    _IMU IMU = _IMU();
    _Motion Motion = _Motion();
    _Button Button = _Button();
    _LineTracker LineTracker = _LineTracker();
    _Light Light = _Light();  
  
    RoboWunderkind();
    void begin();
    void print_attached_modules();
    void enable_power(bool on_off);
    void mac_address();

    void wait_for_all_actions();
    void wait_for_action(uint8_t mod_type, uint8_t mod_id);
    void wait_for_all_triggers();
    void wait_for_trigger(uint8_t mod_type, uint8_t mod_id);
    bool is_attached(uint8_t mod_type, uint8_t id);

    
};

#endif /* MAIN_ROBO_NODE_H_ */