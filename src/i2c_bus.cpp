#include <driver/i2c.h>
#include <Wire.h>
#include "i2c_bus.h"
#include "module_handler.h"

static uint8_t i2c_devices[MAX_DYNAMIC_I2C_ADD+1] = {0};
static uint8_t i2c_devices_new[MAX_DYNAMIC_I2C_ADD+1] = {0};
static uint8_t i2c_devices_disconnection_count[MAX_DYNAMIC_I2C_ADD+1] = {0};

static SemaphoreHandle_t i2c_mutex = NULL;

void reset_i2c_devices()
{
  for(uint8_t i = 0; i < MAX_DYNAMIC_I2C_ADD+1; i ++)
  {
    i2c_devices_disconnection_count[i] = 0;
    i2c_devices[i] = 0;
  }
}

void i2c_confirm_module_existence_in_scanner(uint8_t address)
{
  i2c_devices_new[address] = 2;
}

void init_i2c(void)
{
  vSemaphoreCreateBinary(i2c_mutex);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  //if( i2c_mutex != NULL ) printf("Created Semaphore!\n");
}

// -------------------------------- I2C MODULE COMMUNICATION ---------------------------------------------------

bool i2c_write_module(uint8_t address, uint8_t *data, size_t size)
{
    uint8_t espRc = 0xff;
    if( xSemaphoreTake(i2c_mutex, I2C_WAIT_TICKS_SM) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(data, size);
      espRc = Wire.endTransmission();
      xSemaphoreGive( i2c_mutex );
    }
    else
    {
      //printf("Unable to aquire i2c write\n");
    }
    if(espRc != 0)
    {
      //printf("Bad I2C Transmit\n");
      return false;
    }
    else
    {
      //printf("I2C Transmit OK\n");
      return true;
    }
}

uint8_t i2c_read_module_data(uint8_t i2c_address, uint8_t* data_rd, size_t size)
{ 
    uint8_t i = 0;
    if( xSemaphoreTake(i2c_mutex, I2C_WAIT_TICKS_SM) == pdTRUE)
    {
      Wire.requestFrom(i2c_address, size);
      while ((Wire.available()) && (i <= size))  
      { 
        int c = Wire.read();    
        data_rd[i] = c; 
        //printf("I2C: Read Data - 0x%02X\n", data_rd[i]);
        i++;
      }
      //printf("I2C: Read - DONE\n");
      xSemaphoreGive(i2c_mutex);
      //if(i != 0) printf("I2C Read Data Successful\n");
      //else printf("I2C Read Module Failed\n");
      return i;
    }
    else
    {
      //printf("Unable to aquire i2c read data\n");
    }
    return i;
}

uint8_t i2c_read_module_info(uint8_t i2c_address, uint8_t code, uint8_t* data_rd, size_t size)
{
    uint8_t i = 0;
    if( xSemaphoreTake(i2c_mutex, I2C_WAIT_TICKS_SM) == pdTRUE)
    {
      Wire.beginTransmission(i2c_address);
      Wire.write(code);
      Wire.endTransmission();
    
      Wire.requestFrom(i2c_address, size);
      while ((Wire.available()) && (i <= size))  
      { 
        int c = Wire.read();    
        data_rd[i] = c; 
        //printf("I2C: Read Info - 0x%02X\n", data_rd[i]);
        i++;
      }
      //printf("I2C: Read - DONE\n");
      xSemaphoreGive(i2c_mutex);
      //if(i != 0) printf("I2C Read Info Successful\n");
      //else printf("I2C Read Info Failed\n");
      return i;
    }
    else
    {
      //printf("Unable to aquire i2c read info\n");
    }
    return i;
}

uint8_t i2c_eeprom_module_id(uint8_t address)
{
  uint8_t data[3] = {0};
  i2c_read_module_info(address, 0x05, data, 3);
  return data[0];
}

void i2c_set_modules_address(uint8_t old_address, uint8_t bitpos, uint8_t new_address1, uint8_t new_address2)
{
  uint8_t data[5];
  data[0] = 0x01;
  data[1] = 0x00;

  i2c_write_module(old_address, data, 2);

  data[0] = 0x04;
  data[1] = bitpos;
  data[2] = new_address1;
  data[3] = new_address2;

  i2c_write_module(old_address, data, 4);
}

void i2c_scan(void)
{
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t conflict_add_count = 0;
  uint8_t data_rd[25] = {0};
  uint8_t module_counter = 0;
  uint8_t error;

  uint8_t bitpos = 0;
  uint8_t new_add0 = 0;
  uint8_t new_add1 = 0;
  uint32_t bitmask = 0;
  uint32_t modules_id = 0;
  uint32_t modules_id_n = 0;
  uint8_t data[3] = {0};

  // Check the whole i2c address range for connect or disconnect events.
  for (i = BOOTLOADER_I2C_ADD; i <= MAX_DYNAMIC_I2C_ADD; i++)
  {
    usleep(40); 
    Wire.beginTransmission(i);
    error = Wire.endTransmission();
   

    if (error == 0)  //module found at i.
    {
      //usleep(40); // 4 i2c cycles 40 only sleep after we get a response
      if((i2c_devices[i] == ADD_DETECTION_CYCLES) && (i >= MIN_DYNAMIC_I2C_ADD) && (i <= MAX_DYNAMIC_I2C_ADD))
      {
        /*
         * This can happen if the module could not be detected in one cycle but reappears in the next, so its existence is
         * confirmed and the module disconnection cycle is interrupted.
         */
        //printf("SCANNER: Confirming 0x%02x\n", i);
        module_counter ++;
        confirm_modules_existance(i);
        i2c_devices_disconnection_count[i] = 0;
        continue;
      }
      // Module counts as connected as soon as i2c_devices_new[i] is = ADD_DETECTION_CYCLES, so it needs to be detected for at least two cycles.
      else if (i2c_devices[i] != ADD_DETECTION_CYCLES)
      {
        i2c_devices_new[i]++;
        //printf("Connection Count of 0x%02x = %d\n", i, i2c_devices_new[i]);
      }
    }
    else
    { 
      //i2c_devices_new[i] = 0;
      
      // If this module is connected increment disconnect count
      if(i2c_devices[i] == ADD_DETECTION_CYCLES) i2c_devices_disconnection_count[i]++;
      // else we are dealing with a module we have not discovered, move on
      else
      {
        i2c_devices[i] = i2c_devices_new[i] = 0;// this is not a confirmed module, and we have failed to contact it so we will clear its values
        continue; // else we are dealing with a module we have not discovered, move on
      }

      //printf("Disconnection Count of 0x%02x = %d\n", i, i2c_devices_disconnection_count[i]);
      // If our disconnect count is over the threshold then reset our detection count so we can begin disconnect routine
      if(i2c_devices_disconnection_count[i] >= DELETE_DETECTION_CYCLES)
      {
        i2c_devices_new[i] = 0; // reset the detection count
        i2c_devices_disconnection_count[i] = 0;
      } 
    }
  }
  //printf("SCANNER: Total Connected Modules = %d\n", module_counter);
  module_counter = 0;

  // After checking for changes on the bus, handle changes according to the address range and the module state.
  for (i = BOOTLOADER_I2C_ADD; i <= MAX_DYNAMIC_I2C_ADD; i++)
  {
    //usleep(40); // 2 i2c cycles
    if (i2c_devices_new[i] != i2c_devices[i])
    {
      // Means that a module has been confrimed on the bus.
      if (i2c_devices_new[i] == ADD_DETECTION_CYCLES)
      {
        //usleep(40);
        //printf("SCANNER: Module detected twice at 0x%02x\n", i);
        if (i == BOOTLOADER_I2C_ADD)
        {
          //printf("SCANNER: Bootloader detected\n");
        }

        /*
         * In this address range the module has not been initialised and added to the system yet.
         * The module, when powered, appears on the bus with a random address in the RANDOM_I2C_ADD range.
         * The system will check if there is an address conflict, if yes, assign the modules two different addresses
         * out of the CONFLICT_SOLVING_I2C_ADD range and continue to do so as long as there is only one module at
         * each address. The last step is to assign the module its address out of the DYNAMIC_I2C_ADD range, initialize
         * it and add it to the system.
         */
        if (i >= MIN_RANDOM_I2C_ADD && i <= MAX_CONFLICT_SOLVING_I2C_ADD)
        {
          // Manipulate module out buffer to check for an address conflict.
          i2c_read_module_data(i, data_rd, 9);
          modules_id = (uint8_t)data_rd[1] | ((uint8_t)data_rd[3] << 8) | ((uint8_t)data_rd[5] << 16) | ((uint8_t)data_rd[7] << 24);
          modules_id_n = ((uint8_t)(data_rd[2] ^ 0xff)) | ((uint8_t)(data_rd[4] ^ 0xff) << 8) | ((uint8_t)(data_rd[6] ^ 0xff)<< 16) | ((uint8_t)(data_rd[8] ^ 0xff)<< 24);

          //printf("SCANNER: modules_id = 0x%02X & modules_id_n = 0x%02X\n", modules_id, modules_id_n);

          // Check for address conflict
          if ((data_rd[1] != (data_rd[2] ^ 0xff)) || (data_rd[3] != (data_rd[4] ^ 0xff)) || (data_rd[5] != (data_rd[6] ^ 0xff)) || (data_rd[7] != (data_rd[8] ^ 0xff)))
          {
            bitpos = 0;
            new_add0 = 0;
            new_add1 = 0;
            bitmask = 0;

            // Find the first different bit in the data array manipulated above, needed to assign module solving addresses to the two modules on the same address.
            for (j = 0; j < 32; j++)
            {
              bitmask = (1 << j);
              if ((modules_id ^ modules_id_n) & bitmask)
              {
                bitpos = j;
                break;
              }
            }
            // Find next free module solving address.
            new_add0 = MIN_CONFLICT_SOLVING_I2C_ADD + conflict_add_count;
            conflict_add_count++;
            // Find next free module solving address.
            new_add1 = MIN_CONFLICT_SOLVING_I2C_ADD + conflict_add_count;
            conflict_add_count++;
            if((conflict_add_count + MIN_CONFLICT_SOLVING_I2C_ADD) ==  MAX_CONFLICT_SOLVING_I2C_ADD) conflict_add_count  = 0;

            /*
             * Assign module solving addresses to the two modules on the same address.
             * In case there are more then two modules on the same address, two will reappear on the bus with the same conflict solving address,
             * the system will detect this again and assign them new conflict solving addresses.
             */
            //printf("SCANNER: Address conflict detected at 0x%02x, bit %d, trying to solve, new addresses are 0x%02x and 0x%02x\n", i, bitpos, new_add0, new_add1);
            i2c_set_modules_address(i, bitpos, new_add0, new_add1);
          }
          else
          {
            //If there is no address conflict, the module will be added to the system.
            if(i2c_devices[i] == ADD_DETECTION_CYCLES - 1)
            {
              //printf("SCANNER: Adding module at 0x%02x, unique ID = %d\n", i, modules_id);
              i2c_read_module_info(i, 0x01, data_rd, 9);
              uint8_t eeprom = i2c_eeprom_module_id(i);
              add_module(i, eeprom, data_rd);
            }
          }
        }
        if ((i >= MIN_DYNAMIC_I2C_ADD) && (i <= MAX_DYNAMIC_I2C_ADD))
        {
          /*
           * This can happen if the module could not be detected in one cycle but reappears in the next, so its existence is
           * confirmed and the module disconnection cycle is interrupted.
           */
          //printf("SCANNER: Confirming 0x%02x\n", i);
          confirm_modules_existance(i);
          i2c_devices_disconnection_count[i] = 0;
        }
      }
      else if (i2c_devices_new[i] == 0)
      {
        // The module disappeared from the bus, the disconnection cycle is initiated.
        // Only remove if it is a real module that is currently connected
        if ((i > MAX_CONFLICT_SOLVING_I2C_ADD) && i2c_devices[i] == ADD_DETECTION_CYCLES)
        { 
          initiate_module_disconnection(i);
          //printf("SCANNER: Module disconnected at 0x%02x, dev is %d\n", i, i2c_devices[i]);
        }
      }
      else
      {
        //printf("SCANNER: New module detected at 0x%02x\n", i);
      }
    }
    i2c_devices[i] = i2c_devices_new[i];
  }
}
