#ifndef MAIN_I2C_BUS_H
#define MAIN_I2C_BUS_H

#include "Arduino.h"
#include <cstddef>

#include "module_handler.h"

// ------------------------------- MODULE CONSTANTS -------------------------------------------------------------

#define BOOTLOADER_I2C_ADD              0x08      // Bootloader is always at this address
#define MIN_RANDOM_I2C_ADD              0x0A      // Random address pool for modules after boot
#define MAX_RANDOM_I2C_ADD              0x19
#define MIN_CONFLICT_SOLVING_I2C_ADD    0x1A      // Conflict solving address pool
#define MAX_CONFLICT_SOLVING_I2C_ADD    0x2F
#define MIN_DYNAMIC_I2C_ADD             MOTOR1_ADD           //0x30
#define MAX_DYNAMIC_I2C_ADD             MAX_I2C_ADDR-1

#define I2C_SDA_PIN                     22
#define I2C_SCL_PIN                     21

#define I2C_DATA_LENGTH                 512             // Data buffer length for test buffer
#define I2C_ACK_CHECK_EN                0x1             // I2C master will check ack from slave
#define I2C_ACK_CHECK_DIS               0x0             // I2C master will not check ack from slave
#define I2C_ACK_VAL                     0x0             // I2C ack value
#define I2C_NACK_VAL                    0x1             // I2C nack value

#define ADD_DETECTION_CYCLES            3               // 2
#define DELETE_DETECTION_CYCLES         2               // 1

#define STANDARD_FRQUENCY               100000
#define I2C_WAIT_TICKS                  100/portTICK_PERIOD_MS//10/portTICK_PERIOD_MS
#define I2C_WAIT_TICKS_SM               250/portTICK_PERIOD_MS//10/portTICK_PERIOD_MS
//#define DETECTION_CYCLES              3

// ----------------  FUNCTIONS:  --------------

void init_i2c(void);

/*
 *  Called in main while(1) loop.
 *  Scans I2C bus from address BOOTLOADER_I2C_ADD to MAX_DYNAMIC_I2C_ADD, updates "i2c_devices_new" according to found modules.
 *  Compares "i2c_devices_new" to "i2c_devices", if a new module has been detected the address is checked for an address conflict.
 *  If yes the address conflict gets solved by giving the modules a address conflict solving address.
 *  Adds the new module to the currently attached modules, deletes a deleted module from the currently attached modules.
 *
 *  Initializes the local I2C bus as well which is responsible for communicating with the PMIC
 *
 *  Parameters:
 *    -
 *
 *  Return value:
 *    -
 */
void i2c_scan(void);

/*
 *  Confirms modules existence by overwriting "i2c_devices_new[]" array.
 *  Needed if user plugs and unplugs a module before the i2c scanner fully registers module but bitmask change
 *  has already been sent to the host.
 *
 *  Parameters:
 *    - uint8_t address: modules address to confirm.
 *
 *  Return value:
 *    -
 */
void i2c_confirm_module_existence_in_scanner(uint8_t address);

/*
 *  Sets a modules i2c address to a new one.
 *
 *  Parameters:
 *    - uint8_t old_address: current addres of the module that needs an address change.
 *    - uint8_t bitpos: needed in case of an address conflict, see "i2c_scan" function in "i2c_communication.c"
 *    - uint8_t new_address1: new address to set
 *    - uint8_t new address2: in case there is an address conflict at "old_address", set this to a different address then
 *                "new_address1", if not, set same as new_address1".
 *
 *  Return value:
 *    -
 */
void i2c_set_modules_address(uint8_t old_address, uint8_t bitpos, uint8_t new_address1, uint8_t new_address2);

/*
 *  Used to read the modules info, like type, fw version, ...
 *  Sends 0x00 0x01 before reading modules i2c out buffer
 *
 *  Parameters:
 *    - uint8_t address: modules address.
 *    - uint8_t* data: pointer to write the data that has been read.
 *    - size_t size: number of bytes to read from modules buffer.
 *
 *  Return value:
 *    -
 */
uint8_t i2c_read_module_info(uint8_t address, uint8_t code, uint8_t* data_rd, size_t size);
/*
 *  Used to read the modules i2c out buffer.
 *
 *  Parameters:
 *    - uint8_t address: modules address.
 *    - uint8_t* data: pointer to write the data that has been read.
 *    - size_t size: number of bytes to read from modules buffer.
 *
 *  Return value:
 *    - uint8_t size: number of bytes that have been read, 0 if reading failed.
 */
uint8_t i2c_read_module_data(uint8_t address, uint8_t* data_rd, size_t size);

/*
 *  Used to write to the modules i2c in buffer
 *
 *  Parameters:
 *    - uint8_t address: modules address.
 *    - uint8_t* data: pointer to write the data that needs to be written.
 *    - size_t size: number of bytes to write to the modules in buffer.
 *
 *  Return value:
 *    - uint8_t size: number of bytes that have been written.
 */
bool i2c_write_module(uint8_t address, uint8_t *data, size_t size);

void reset_i2c_devices();


#endif
