#ifndef MAIN_MODULE_HANDLER_H_
#define MAIN_MODULE_HANDLER_H_

#include "Arduino.h"

#define MODULE_DISCONNECTING                1
#define MODULE_DISCONNECTED                 2
#define MODULE_CONNECTED                    3

#define ID_NOT_EXISTING                     0
#define ID_EXISTING                         1

#define MODULES_FIRST_ACTION_OR_TRIGGER     0
#define MODULES_SECOND_ACTION_OR_TRIGGER    1

#define NUMBER_OF_POSSIBLE_MOTORS           6

#define MODULE_FW_OUTDATED                  1
#define MODULE_FW_UP_TO_DATE                0

#define ACTION_NOT_SET                     -1
#define ACTION_ID                           1
#define TRIGGER_ID                          2

#define MODULE_64_CHUNKS                    2

#define SERVOV2       0   
#define BUTTON        2   
#define RGB_LED       3   
#define MATRIX        4   
#define MOTOR         5   
#define SERVOV        6   
#define ULTRASONIC    7  
#define MOTION        8
#define LINETRACKER   9   
#define SYSTEM        10   
#define LIGHTSENSOR   11
#define ACCELEROMETER 15   
#define CLAW          16
#define HINGE         17
#define DISPLAY       18
#define KNOB          19

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

// MODULE TYPES:
#define MODULE_SERVO_V2         0x00
#define MODULE_BUTTON           0x02
#define MODULE_RGB              0x03
#define MODULE_MATRIX           0x04
#define MODULE_MOTOR            0x05
#define MODULE_SERVO            0x06
#define MODULE_ULTRASONIC       0x07
#define MODULE_PIR              0x08
#define MODULE_LINETRACKER      0x09
#define MODULE_SYSTEM           0x0A
#define MODULE_LIGHTSENSOR      0x0B
#define MODULE_IRBLASTER        0x0D
#define MODULE_LINETRACK_COLOUR 0x0E
#define MODULE_ACCELEROMETER    0x0F
#define MODULE_CLAW             0x10
#define MODULE_HINGE            0x11
#define MODULE_DISPLAY          0x12
#define MODULE_BUMP             0x13
#define MODULE_KNOB             0x14
#define NUM_MODULE_TYPES        0x15     // Total number of unique modules
#define MODULE_BOOTLOADER       0xFE   

#define SYSTEM_BUILD            32768

// MODULES ADDRESSES: Must be consecutive!

typedef enum
{
  MOTOR1_ADD = 0x30,
  MOTOR2_ADD,
  MOTOR3_ADD,
  MOTOR4_ADD,
  MOTOR5_ADD,
  MOTOR6_ADD,
  SERVO1_ADD,
  SERVO2_ADD,
  SERVO3_ADD,
  SERVO4_ADD,
  SERVO5_ADD,
  SERVO6_ADD,
  RGB1_ADD,
  RGB2_ADD,
  RGB3_ADD,
  RGB4_ADD,
  RGB5_ADD = 0x40,
  RGB6_ADD,
  RGB7_ADD,
  RGB8_ADD,
  ULTRASONIC1_ADD,
  ULTRASONIC2_ADD,
  ULTRASONIC3_ADD,
  ULTRASONIC4_ADD,    
  BUTTON1_ADD,
  BUTTON2_ADD,
  BUTTON3_ADD,
  BUTTON4_ADD,
  MATRIX1_ADD,
  MATRIX2_ADD,
  MATRIX3_ADD,
  MATRIX4_ADD,
  MATRIX5_ADD = 0x50, 
  MATRIX6_ADD,
  MATRIX7_ADD,
  MATRIX8_ADD,
  LIGHTSENSOR1_ADD,
  LIGHTSENSOR2_ADD,
  LIGHTSENSOR3_ADD,
  LIGHTSENSOR4_ADD,
  PIR1_ADD,
  PIR2_ADD,
  PIR3_ADD,
  PIR4_ADD,
  LINETRACKER1_ADD,
  LINETRACKER2_ADD,
  LINETRACKER3_ADD,
  LINETRACKER4_ADD,
  ACCELEROMETER1_ADD = 0x60,
  ACCELEROMETER2_ADD,
  ACCELEROMETER3_ADD,
  ACCELEROMETER4_ADD,
  DISPLAY1_ADD,
  DISPLAY2_ADD,
  DISPLAY3_ADD,
  DISPLAY4_ADD,
  HINGE1_ADD,
  HINGE2_ADD,
  HINGE3_ADD,
  HINGE4_ADD,
  CLAW1_ADD,
  CLAW2_ADD,
  CLAW3_ADD,
  CLAW4_ADD,
  PROPER_SERVO1_ADD = 0x70,
  PROPER_SERVO2_ADD,
  PROPER_SERVO3_ADD,
  PROPER_SERVO4_ADD,
  KNOB1_ADD,
  KNOB2_ADD,
  KNOB3_ADD,
  KNOB4_ADD,
  MAX_I2C_ADDR  // 0x78
} module_addresses;

// BITMASK FOR CONNECTED MODULES:
typedef enum
{
  MOD_MOTOR1 = 0,
  MOD_MOTOR2,
  MOD_MOTOR3,
  MOD_MOTOR4,
  MOD_SERVO1,
  MOD_SERVO2,
  MOD_LEDRGB1,
  MOD_LEDRGB2,  //= 7,
  MOD_MATRIX1,
  MOD_IR1,
  MOD_CAMERA1,
  MOD_BUTTON1,
  MOD_BUTTON2,
  MOD_LIGHTSENSOR1,
  MOD_METEO1,
  MOD_SYSTEM,    // = 15,
  MOD_ULTRASONIC1,
  MOD_PIR1,
  MOD_MOTOR5,
  MOD_MOTOR6,
  MOD_SERVO3,
  MOD_SERVO4,
  MOD_SERVO5,
  MOD_SERVO6, // = 23,
  MOD_LEDRGB3,
  MOD_LEDRGB4,
  MOD_LEDRGB5,
  MOD_LEDRGB6,
  MOD_LEDRGB7,
  MOD_LEDRGB8,
  MOD_MATRIX2,
  MOD_MATRIX3, // = 31,
  MOD_MATRIX4,
  MOD_MATRIX5,
  MOD_MATRIX6,
  MOD_MATRIX7,
  MOD_MATRIX8,
  MOD_BUTTON3,
  MOD_BUTTON4,
  MOD_LIGHTSENSOR2, // = 39,
  MOD_LIGHTSENSOR3,
  MOD_LIGHTSENSOR4,
  MOD_ULTRASONIC2,
  MOD_ULTRASONIC3,
  MOD_ULTRASONIC4,
  MOD_PIR2,
  MOD_PIR3,
  MOD_PIR4, // = 47,
  MOD_LINETRACKER1,
  MOD_LINETRACKER2,
  MOD_LINETRACKER3,
  MOD_LINETRACKER4,
  MOD_ACCELEROMETER1,
  MOD_ACCELEROMETER2,
  MOD_ACCELEROMETER3,
  MOD_ACCELEROMETER4, // = 55,
  MOD_DISPLAY1,
  MOD_DISPLAY2, 
  MOD_DISPLAY3, 
  MOD_DISPLAY4, 
  MOD_HINGE1,
  MOD_HINGE2,
  MOD_HINGE3,
  MOD_HINGE4, // = 63,
  MOD_CLAW1,
  MOD_CLAW2,
  MOD_CLAW3,
  MOD_CLAW4,
  MOD_PROPER_SERVO1,
  MOD_PROPER_SERVO2,
  MOD_PROPER_SERVO3,
  MOD_PROPER_SERVO4, 
  MOD_KNOB1, 
  MOD_KNOB2, 
  MOD_KNOB3, 
  MOD_KNOB4, 
  MODULES_NUMBER 
} module_instance; // length is 76

#define SAME_TYPE_MAX     8
#define MAX_SYSTEMS       1 
#define MAX_MOTORS        6
#define MAX_SERVOS        6
#define MAX_RGBS          8
#define MAX_MATRIX        8
#define MAX_BUTTONS       4
#define MAX_PIRS          4
#define MAX_ACC           4
#define MAX_ULTRAS        4
#define MAX_LIGHT         4
#define MAX_LINES         4
#define MAX_PRS           4
#define MAX_DISPLAYS      4
#define MAX_HINGES        4
#define MAX_CLAWS         4
#define MAX_KNOBS         4

typedef   bool (*Reset)(uint8_t module_index);
typedef   bool (*Interrupt_Check)(uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2);

typedef struct{
  uint8_t max_modules_of_type;
  uint8_t type;
  uint8_t ids[SAME_TYPE_MAX];           // module id
  uint8_t addresses[SAME_TYPE_MAX];      // assigned address
  int8_t action_trigger_ids[SAME_TYPE_MAX][2]; // action or trigger id
  uint8_t module_enum[SAME_TYPE_MAX];
  uint8_t connections[SAME_TYPE_MAX];
  int16_t modules_disconnection_count[SAME_TYPE_MAX];
  Reset reset;
  Interrupt_Check check;
}Module_Type;

uint8_t modules_address_to_enum(uint8_t address);
uint8_t modules_address_to_type(uint8_t address);
uint8_t modules_enum_to_address(uint8_t modules_enum);
uint8_t modules_type_and_index_to_enum(uint8_t type, uint8_t index);
void reset_drive_motors(uint8_t action_id);
uint8_t get_linetracker_action_status();
uint8_t get_audioplayer_action();
bool check_if_attached_from_index(uint8_t module_type, uint8_t module_index);
bool check_module_trigger_status(uint8_t mod_type, uint8_t mod_id);
bool check_module_action_status(uint8_t mod_type, uint8_t mod_id);
bool check_all_actions();

void set_module_action_trigger_id(uint8_t mod_type, uint8_t mod_id, uint8_t action_trigger_id, uint8_t module_action_or_trigger_number);
void print_module_config();
void print_pneumonic_modules();

void reset_module_connections();

/*
 *  Returns true if the module is attached
 */
bool check_if_module_attached(uint8_t mod_enum);

/*
 *  Returns a bitmask representing the currently connected modules, see layout above.
 *
 *  Parameters:
 *    -
 *
 *  Return value:
 *    - uint64_t modules: bitmask, connected modules.
 */
//uint64_t get_connected_modules_bitmask(void);

/*
 *  Adds a new module to the connected modules bitmask.
 *  Sets modules address, see above.
 *  Sends updated connected modules bitmask to host.
 *  The modules unique id is used to detect reconnecting modules and give them the
 *  same address the had before.
 *
 *  Parameters:
 *    - uint8_t address: modules address as it was initially detected
 *    - uint32_t module_id: module identifier, unique for every microcontroller in the modules
 *    - uint8_t* buf: modules data, holds type, fw version...
 *
 *  Return value:
 *    -
 */
void add_module(uint8_t address, uint8_t eeprom_id, uint8_t* buf);

/*
 *  Deletes an existing module from the connected modules bitmask.
 *  Sends updated connected modules bitmask to host.
 *
 *  Parameters:
 *    - uint8_t address: modules address
 *
 *  Return value:
 *    -
 */
void delete_module(uint8_t address);

/*
 *  Clears the connected module count, "modules_count" struct, see in "modules.c".
 *
 *  Parameters:
 *    -
 *
 *  Return value:
 *    -
 */
void init_modules(void);

/*
 *  For motors: if a motor is connected, uses "reset_module_via_enum" to reset it.
 *  For all other modules: if their first or second action is set, uses "reset_module_via_enum" to reset it.
 *
 *  Parameters:
 *    -
 *
 *  Return value:
 *    -
 */
void reset_all_modules(void);

/*
 *  Prints connected modules, for debugging.
 *
 *  Parameters:
 *    -
 *
 *  Return value:
 *    -
 */
void print_connected_modules(void);

/*
 *  If a modules state changes from CONNECTED to DISCONNECTING, but then the i2c scanner finds it again,
 *  this function is called and the disconnection cycle is interrupted.
 *  finds it again,
 *
 *  Parameters:
 *    - uint8_t address: address of the module
 *
 *  Return value:
 *    -
 */
void confirm_modules_existance(uint8_t address);

/*
 *  Called by main in the while(1) loop.
 *  Checks if the state of a module is DISCONNECTING, if yes increases a disconnection counter
 *  for the module. If the module stays in DISCONNECING mode for five main cycles, the module is deleted
 *  and its state changes to DISCONNECTED.
 *
 *  Parameters:
 *    -
 *
 *  Return value:
 *    -
 */
void module_disconnection_handler(void);

/*
 *  Changes the modules state from CONNECTED to DISCONNECTING.
 *  Called by "i2c_scan" if the modules address cannot be detected on the i2c bus anymore.
 *
 *  Parameters:
 *    - uint8_t address: address of the module
 *
 *  Return value:
 *    -
 */
void initiate_module_disconnection(uint8_t address);

/*
 *  Called by main in the while(1) loop.
 *  Checks if an action or a trigger has been set for a module, if yes, reads the modules i2c out buffer and
 *  checks if the module has reset its action state flag, if yes, calls "reset_modules_action_or_trigger" for the module.
 *  Checks also the audio playback status, in case there is a set action for the system cube.
 *
 *  Parameters:
 *    -
 *
 *  Return value:
 *    -
 */
void read_module_events(void);

/*
 *  If an action or a trigger is set for a module, the system keeps track of this by setting the corresponding position
 *  in the "modules_first_action_state[]" or "modules_second_action_state[]" arrays (used for modules that have more then
 *  one action or trigger, for example ultrasonic cube: distance trigger and sound trigger).
 *  The position will be changed from -1 (no action or trigger set) to "action_or_trigger_id". The id is given by the
 *  host when setting an action or trigger and will be sent back to it when an action or trigger has been completed or has occurred.
 *
 *  Parameters:
 *    - uint8_t modules_enum
 *    - uint8_t action_or_trigger_id: id, sent by host
 *    - uint8_t modules_action_or_trigger_number: MODULES_FIRST_ACTION_OR_TRIGGER or MODULES_SECOND_ACTION_OR_TRIGGER
 *
 *  Return value:
 *    -
 */
void set_modules_action_or_trigger(uint8_t modules_enum, uint8_t action_or_trigger_id, uint8_t modules_action_or_trigger_number);

/*
 *  Clears "modules_first_action_state[]" or "modules_second_action_state[]".
 *  Sends a "CMD_ACTION_OR_TRIGGER_RESPONSE" message to the host, containing the "action_or_trigger_id" saved when setting
 *  the aciton or trigger. Resets the corresponding position in the array to -1.
 *
 *
 *  Parameters:
 *    - uint8_t modules_enum
 *    - uint8_t modules_action_or_trigger_number: MODULES_FIRST_ACTION_OR_TRIGGER or MODULES_SECOND_ACTION_OR_TRIGGER
 *
 *  Return value:
 *    -
 */
bool reset_modules_action_or_trigger(uint8_t modules_enum, uint8_t modules_action_or_trigger_number);

/*
 *  "audio_action_status" is used to keep track of audio playback actions.
 *  Called in "audio_player.c" if the playback of a sample is done.
 *  Called by command handler if a audio playback action is set.
 *
 *  Parameters:
 *    - uint8_t playerstatus: idle, busy
 *
 *  Return value:
 *    -
 */
void set_audioplayer_action_status(uint8_t playerstatus);

/*
 *  "drive_action_status" is used to keep track of the drive action.
 *  The drive action can involve up to 6 motors. In order to send a "CMD_ACTION_OR_TRIGGER_RESPONSE" only after
 *  all involved motors are done, the "drive_action_status" when set, saves a bitmask of all involved motors.
 *  With eeach motor that is done with the action, the corresponding bit is cleared in the bitmask.
 *  After the bitmask is 0, the "CMD_ACTION_OR_TRIGGER_RESPONSE" is sent to the host.
 *  Called by command handler if a drive function is set.
 *
 *
 *  Parameters:
 *    - uint8_t motors_bitfield: active motors bitmask: bit 0 is 1 if motor 1 is involved, ...
 *
 *  Return value:
 *    -
 */
void set_drive_action_status(uint8_t motors_bitfield);

/*
 *  "linetracker_action_status" is used to keep track of the linetracker aciton.
 *  Called in "line_tracking.c" if the linetracker aciton has been stopped.
 *  Called by command handler if a linetracker action is set.
 *
 *  Parameters:
 *    -
 *
 *  Return value:
 *    -
 */
void set_linetracker_action_status(uint8_t trackerstatus);

/*
 *  Called on BLE connection (when host requests system cube fw version).
 *  Checks if a module is connected and if the corresponding position in "modules_firmware_out_of_date_record[]"
 *  is set. If yes, sends a message to the host, saying that a out of date module is connected.
 *  The host then can send back a request to update the modules firmware.
 *
 *  Parameters:
 *    -
 *
 *  Return value:
 *    -
 */
void check_all_connected_modules_firmware_version(void);

/*
 *  At connection, every module is checked for its firmware version and compared to the latest versio
 *  specified in "main.h". If the firmware is out of date the corresponding position in "modules_firmware_out_of_date_record[]"
 *  is set. Once the module has been successfully updated or has been disconnected, the record will be reset.
 *
 *  Parameters:
 *    - uint8_t modules_enum
 *    - uint8_t state: MODULE_FW_OUTDATED or MODULE_FW_UP_TO_DATE
 *
 *  Return value:
 *    -
 */
void set_modules_firmware_out_of_date_record(uint8_t modules_enum, uint8_t state);

/*
 *  Checks a given firmware version agains what is specified in "main.h"
 *  Returns the outcome of the check.
 *
 *  Parameters:
 *    - int8_t FW_minor: minor firmware version of the module to be checked
 *    - int8_t FW_major: major firmware version of the module to be checked
 *    - int8_t HW: hardware version of the module to be checked
 *    - uint8_t module_type)
 *
 *  Return value:
 *    - uint8_t result: MODULE_FW_OUTDATED or MODULE_FW_UP_TO_DATE
 */
uint8_t check_module_fw_version(int8_t FW_minor, int8_t FW_major, int8_t HW, uint8_t module_type);

void set_modules_status_from_address(uint8_t addr, uint8_t value);

/*
 * 
 *
 *  Parameters:
 *    - uint8_t module_enum)
 *
 *  Return value:
 *    - uint8_t result: Module status MODULE_CONNECTED, MODULE_DISCONNECTED, MODULE_DISCONNECTING
 */
uint8_t get_module_status(uint8_t modules_enum);

/*
 *  Return value:
 *    - uint64_t The configuation bitmask of Robo
 */
void get_build(uint64_t* current_bitmask);

/*
 *  Sets the old configuration bitmask = to current configuration bitmask
 */
void save_build(void);

/*
 *  Check the difference between what Robo was configured before
 *
 *
 *  Return value:
 *    - int8_t result: Number of modules that have changed since last time we checked
 */
int8_t module_difference(void);

/*
 *  Send the robot configuration to the app
 */
void send_module_configuration();

/*
 *  Checks the current configuration to last configuration sent, if they're different then send the configuration to the app
 */
void check_configuration(void);







#endif
