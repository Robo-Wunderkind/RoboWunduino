
#include "i2c_bus.h"
#include "module_functions.h"
#include "module_handler.h"

uint64_t modules_bitmask[MODULE_64_CHUNKS] = {SYSTEM_BUILD, 0};
uint64_t modules_bitmask_old[MODULE_64_CHUNKS] = {SYSTEM_BUILD, 0};
uint8_t  drive_action_status = 0;

Module_Type modules[NUM_MODULE_TYPES];

static char* LABELS[]=  
{"Servo2","N/A","Button","RGB","Matrix","Motor","Servo","Ultrasonic","Motion","Line Tracker","System","Light","N/A","N/A","N/A","Accelerometer","Claw","Hinge","Display","N/A","Knob"};

static const uint8_t SYSTEM_ENUMS[MAX_SYSTEMS]       = {MOD_SYSTEM};
static const uint8_t MOTOR_ENUMS[MAX_MOTORS]         = {MOD_MOTOR1, MOD_MOTOR2, MOD_MOTOR3, MOD_MOTOR4, MOD_MOTOR5, MOD_MOTOR6};
static const uint8_t SERVO_ENUMS[MAX_SERVOS]         = {MOD_SERVO1, MOD_SERVO2, MOD_SERVO3, MOD_SERVO4, MOD_SERVO5, MOD_SERVO6};
static const uint8_t RGB_ENUMS[MAX_RGBS]             = {MOD_LEDRGB1, MOD_LEDRGB2, MOD_LEDRGB3, MOD_LEDRGB4, MOD_LEDRGB5, MOD_LEDRGB6, MOD_LEDRGB7, MOD_LEDRGB8};
static const uint8_t MATRIX_ENUMS[MAX_MATRIX]        = {MOD_MATRIX1, MOD_MATRIX2, MOD_MATRIX3, MOD_MATRIX4, MOD_MATRIX5, MOD_MATRIX6, MOD_MATRIX7, MOD_MATRIX8};
static const uint8_t BUTTON_ENUMS[MAX_BUTTONS]       = {MOD_BUTTON1, MOD_BUTTON2, MOD_BUTTON3, MOD_BUTTON4};
static const uint8_t PIR_ENUMS[MAX_PIRS]             = {MOD_PIR1, MOD_PIR2, MOD_PIR3, MOD_PIR4};
static const uint8_t ACC_ENUMS[MAX_ACC]              = {MOD_ACCELEROMETER1, MOD_ACCELEROMETER2, MOD_ACCELEROMETER3, MOD_ACCELEROMETER4};
static const uint8_t ULTRASONIC_ENUMS[MAX_ULTRAS]    = {MOD_ULTRASONIC1, MOD_ULTRASONIC2, MOD_ULTRASONIC3, MOD_ULTRASONIC4};
static const uint8_t LIGHTSENSOR_ENUMS[MAX_LIGHT]    = {MOD_LIGHTSENSOR1, MOD_LIGHTSENSOR2, MOD_LIGHTSENSOR3, MOD_LIGHTSENSOR4};
static const uint8_t LINETRACKER_ENUMS[MAX_LINES]    = {MOD_LINETRACKER1, MOD_LINETRACKER2, MOD_LINETRACKER3, MOD_LINETRACKER4};
static const uint8_t PROPER_SERVO_ENUMS[MAX_PRS]     = {MOD_PROPER_SERVO1, MOD_PROPER_SERVO2, MOD_PROPER_SERVO3, MOD_PROPER_SERVO4};
static const uint8_t DISPLAY_ENUMS[MAX_DISPLAYS]     = {MOD_DISPLAY1, MOD_DISPLAY2, MOD_DISPLAY3, MOD_DISPLAY4};
static const uint8_t HINGE_ENUMS[MAX_HINGES]         = {MOD_HINGE1, MOD_HINGE2, MOD_HINGE3, MOD_HINGE4};
static const uint8_t CLAW_ENUMS[MAX_CLAWS]           = {MOD_CLAW1, MOD_CLAW2, MOD_CLAW3, MOD_CLAW4};
static const uint8_t KNOB_ENUMS[MAX_KNOBS]           = {MOD_KNOB1, MOD_KNOB2, MOD_KNOB3, MOD_KNOB4};

static const uint8_t *ENUMS_LIST[NUM_MODULE_TYPES] = {
  PROPER_SERVO_ENUMS, 
  NULL,
  BUTTON_ENUMS, 
  RGB_ENUMS,
  MATRIX_ENUMS,
  MOTOR_ENUMS, 
  SERVO_ENUMS, 
  ULTRASONIC_ENUMS,
  PIR_ENUMS,
  LINETRACKER_ENUMS, 
  SYSTEM_ENUMS,
  LIGHTSENSOR_ENUMS,
  NULL,
  NULL,
  NULL, 
  ACC_ENUMS,
  CLAW_ENUMS, 
  HINGE_ENUMS, 
  DISPLAY_ENUMS,
  NULL, 
  KNOB_ENUMS
};

static const uint8_t MOD_TYPES[NUM_MODULE_TYPES] = {
  MODULE_SERVO_V2, 
  0x01,
  MODULE_BUTTON,
  MODULE_RGB,
  MODULE_MATRIX,
  MODULE_MOTOR,
  MODULE_SERVO, 
  MODULE_ULTRASONIC,
  MODULE_PIR,
  MODULE_LINETRACKER,
  MODULE_SYSTEM,
  MODULE_LIGHTSENSOR,
  0x0C,
  MODULE_IRBLASTER,
  MODULE_LINETRACK_COLOUR,
  MODULE_ACCELEROMETER, 
  MODULE_CLAW, 
  MODULE_HINGE,
  MODULE_DISPLAY,
  0x13,
  MODULE_KNOB
};

static const uint8_t max_mods[NUM_MODULE_TYPES] = {
  MAX_PRS,      // Proper Servos
  0x00,         // UNK 
  MAX_BUTTONS,  // Buttons
  MAX_RGBS,     // LEDs
  MAX_MATRIX,   // Matrix
  MAX_MOTORS,   // Motors
  MAX_SERVOS,   // Servos
  MAX_ULTRAS,   // Ultrasonics
  MAX_PIRS,     // PIR
  MAX_LINES,    // Line Trackers
  MAX_SYSTEMS,  // System
  MAX_LIGHT,    // Light Sensors
  0x00,         // UNK
  0x00,         // IR 
  0x00,         // COLOUR 
  MAX_ACC,      // Accelerometers
  MAX_CLAWS,    // Claws
  MAX_HINGES,   // Hinges
  MAX_DISPLAYS, // Displays
  0x00,
  MAX_KNOBS     // Knobs 
};

static const uint8_t mod_start_addresses[NUM_MODULE_TYPES] = {
  PROPER_SERVO1_ADD, // Proper Servos
  0xFF,
  BUTTON1_ADD, // Buttons
  RGB1_ADD, // LEDs
  MATRIX1_ADD, // Matrix
  MOTOR1_ADD, // Motors
  SERVO1_ADD, // Servos
  ULTRASONIC1_ADD, // Ultrasonics
  PIR1_ADD, // PIR
  LINETRACKER1_ADD, // Line Trackers
  0xFF,       // System
  LIGHTSENSOR1_ADD, // Light Sensors
  0xFF,
  0xFF,
  0xFF,
  ACCELEROMETER1_ADD, // Accelerometers
  CLAW1_ADD,   // Claws
  HINGE1_ADD, // Hinges
  DISPLAY1_ADD, // Displays
  0xFF,
  KNOB1_ADD     // Knobs 
};

static bool (*resets[NUM_MODULE_TYPES]) (uint8_t index) = {
  reset_servo_proper,
  reset_mod,
  reset_button,
  reset_rgb,
  reset_matrix,
  reset_motor,
  reset_servo,
  reset_ultrasonic,
  reset_pir,
  reset_linetracker,
  reset_mod,
  reset_lightsensor,
  reset_mod,
  reset_mod,
  reset_mod,
  reset_accelerometer,
  reset_claw,
  reset_hinge,
  reset_display,
  reset_mod,
  reset_knob
};

static bool (*checks[NUM_MODULE_TYPES]) (uint8_t address, uint8_t enumm, int8_t action_id1, int8_t action_id2) = {
  check_servo2s,
  check_mod,
  check_buttons,
  check_rgbs,
  check_matrices,
  check_motors,
  check_servos,
  check_ultrasonics,
  check_pirs,
  check_linetrackers,
  check_system,
  check_lightsensors,
  check_mod,
  check_mod,
  check_mod,  
  check_accelerometers,
  check_claws,
  check_hinges,
  check_displays,
  check_mod,  
  check_knobs
};

static void print_addresses()
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j ++)
    {
      //printf("MODULE_HANDLER: %02X -> Type %02X\n", modules[i].addresses[j], modules[i].type);
    }
  }
}

static void create_module_structs()
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    modules[i].max_modules_of_type = max_mods[i];
    modules[i].type = MOD_TYPES[i];
    modules[i].reset = resets[i];
    modules[i].check = checks[i];
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      modules[i].ids[j] = j;
      modules[i].addresses[j] = mod_start_addresses[i] + j;
      modules[i].action_trigger_ids[j][0] = -1;
      modules[i].action_trigger_ids[j][1] = -1;
      modules[i].module_enum[j] = ENUMS_LIST[i][j];
      modules[i].modules_disconnection_count[j] = -1;
      if(MOD_TYPES[i] == MODULE_SYSTEM) modules[i].connections[j] = MODULE_CONNECTED;
      else modules[i].connections[j] = MODULE_DISCONNECTED;
    }
  }
  //print_addresses();
}

static bool module_type_and_id_check(uint8_t module_type, uint8_t module_index)
{
  if(module_type >= NUM_MODULE_TYPES) return false;
  if(module_index > modules[module_type].max_modules_of_type) return false; 
  return true;
}


void init_modules(void)
{
  create_module_structs();

  modules_bitmask[0] = SYSTEM_BUILD;
  modules_bitmask[1] = 0;
  modules_bitmask_old[0] = SYSTEM_BUILD;
  modules_bitmask_old[1] = 0;

  reset_i2c_devices();
}

void reset_module_connections()
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if(MOD_TYPES[i] == MODULE_SYSTEM) modules[i].connections[j] = MODULE_CONNECTED;
      else modules[i].connections[j] = MODULE_DISCONNECTED;
    }
  }
}

uint8_t modules_enum_to_address(uint8_t modules_enum)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if(modules[i].module_enum[j] == modules_enum)
      {
        return modules[i].addresses[j];
      }
    }
  }
  return 0;
}

uint8_t modules_address_to_enum(uint8_t address)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if(modules[i].addresses[j] == address)
      {
        return modules[i].module_enum[j];
      }
    }
  }
  return 0;
}

uint8_t modules_address_to_type(uint8_t address)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if(modules[i].addresses[j] == address)
      {
        return modules[i].type;
      }
    }
  }
  return 0;
}

uint8_t modules_type_and_index_to_enum(uint8_t type, uint8_t index)
{
  //printf("MODULE TYPE = %d   MODULE INDEX = %d\n", type, index);
  return modules[type].module_enum[index];
}

int8_t get_modules_action_or_trigger(uint8_t modules_enum, uint8_t modules_action_or_trigger_number)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if(modules[i].module_enum[j] == modules_enum)
      {
        return modules[i].action_trigger_ids[j][modules_action_or_trigger_number];
      }
    }
  }
  return -1;
}

void set_module_action_trigger_id(uint8_t mod_type, uint8_t mod_id, uint8_t action_trigger_id, uint8_t module_action_or_trigger_number)
{
  if(module_type_and_id_check(mod_type, mod_id) == false) return;
  modules[mod_type].action_trigger_ids[mod_id][module_action_or_trigger_number] = action_trigger_id;
}

void set_modules_action_or_trigger_struct(uint8_t modules_enum, uint8_t action_or_trigger_id, uint8_t modules_action_or_trigger_number)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if(modules[i].module_enum[j] == modules_enum)
      {
        modules[i].action_trigger_ids[j][modules_action_or_trigger_number] = action_or_trigger_id;
      }
    }
  }
  //printf("ACTION/TRIGGER IDS = %d, %d\n", get_modules_action_or_trigger(modules_enum, MODULES_FIRST_ACTION_OR_TRIGGER), get_modules_action_or_trigger(modules_enum, MODULES_SECOND_ACTION_OR_TRIGGER));
}

uint8_t get_modules_status_from_enum(uint8_t enumm)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if(modules[i].module_enum[j] == enumm)
      {
        return modules[i].connections[j];
      }
    }
  }
  return MODULE_DISCONNECTED;
}

void set_modules_status_from_address(uint8_t addr, uint8_t value)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if(modules[i].addresses[j] == addr)
      {
        modules[i].connections[j] = value;
        return;
      }
    }
  }
}

void set_modules_action_or_trigger(uint8_t modules_enum, uint8_t action_or_trigger_id, uint8_t modules_action_or_trigger_number)
{
  set_modules_action_or_trigger_struct(modules_enum, action_or_trigger_id, modules_action_or_trigger_number);
}

void set_drive_action_status(uint8_t motors_bitfield)
{
  drive_action_status = motors_bitfield;
  //printf("ACTION HANDLER: drive action status is %d\n", drive_action_status);
}

void save_build(void)
{
  for(uint8_t i = 0; i < MODULE_64_CHUNKS; i++)
  {
    modules_bitmask_old[i] = modules_bitmask[i];
  }
}

// calculates how many modules have been added or removed
int8_t module_difference(void)
{
  uint8_t difference = 0;
  for(uint8_t i = 0; i < MODULE_64_CHUNKS; i ++)
  {
    uint64_t build = modules_bitmask[i];
    uint64_t build_old = modules_bitmask_old[i];

    if(build == build_old) return 0;

    build = build - ((build>>1) & 0x55555555);
    build = ((build & 0x33333333) + ((build>>2) & 0x33333333));
    uint8_t build_count = (((build + (build>>4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    

    build_old = build_old - ((build_old>>1) & 0x55555555);
    build_old = (build_old & 0x33333333) + ((build_old>>2) & 0x33333333);
    uint8_t build_old_count = (((build_old + (build_old>>4)) & 0xF0F0F0F) * 0x1010101) >> 24;

    modules_bitmask_old[i] = modules_bitmask[i];
    difference += build_count - build_old_count;
  }
  //printf("MODULE HANDLER: Modules Difference = %d\n", difference);
  return difference;
}

uint8_t get_module_status(uint8_t modules_enum)
{
  return get_modules_status_from_enum(modules_enum);
}

void reset_module_via_enum(uint8_t modules_enum)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j ++)
    {
      if(modules[i].module_enum[j] == modules_enum)
      {
        uint8_t index = modules[i].ids[j];
        modules[i].reset(index);
      }
    }
  }
}

void reset_all_modules(void)
{
  //printf("MODULE HANDLER: resetting all modules\n");

  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j ++)
    {
      if(modules[i].connections[j] == MODULE_CONNECTED || modules[i].connections[j] == MODULE_DISCONNECTING)
      {
        //printf("MODULE HANDLER: resetting type %d index %d\n", i, j);
        modules[i].reset(modules[i].ids[j]);
      }
      modules[i].action_trigger_ids[j][MODULES_FIRST_ACTION_OR_TRIGGER] = -1;
      modules[i].action_trigger_ids[j][MODULES_SECOND_ACTION_OR_TRIGGER] = -1;
    }
  }
}

bool check_if_attached_from_index(uint8_t module_type, uint8_t module_index)
{
  if(module_type_and_id_check(module_type, module_index) == false) return false;
  
  if(modules[module_type].connections[module_index] == MODULE_CONNECTED || modules[module_type].connections[module_index] == MODULE_DISCONNECTING) return true;
  else return false;
}

bool check_if_module_attached(uint8_t mod_enum)
{
  uint64_t k = 1;
  return(modules_bitmask[mod_enum/64] & k << mod_enum%64);
}

void confirm_modules_existance(uint8_t address)
{
  set_modules_status_from_address(address, MODULE_CONNECTED);
}

void print_module_config()
{
  printf("MODULE HANDLER: configuration changed, bitmask is:\n");
  uint8_t index = 0;
  uint8_t shift = 0;
  for(uint8_t i = 0; i < (MODULE_64_CHUNKS*8); i++)
  {
    if(i == 8) printf("\n");
    index = i/8;
    shift = (i%8)*8;
    printf(" %c%c%c%c%c%c%c%c ", BYTE_TO_BINARY((uint8_t)(modules_bitmask[index] >> shift)));
  }
  printf("\n");
}

void print_pneumonic_modules()
{
  bool label_set = false;
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j ++)
    {
      if(modules[i].connections[j] == MODULE_CONNECTED || modules[i].connections[j] == MODULE_DISCONNECTING)
      {
        if(!label_set) 
        {
          printf("%s IDs: ", LABELS[i]);
          label_set = true;
        }
        printf(" %d ", j);
      }
      if(j == modules[i].max_modules_of_type - 1 && label_set) printf("\n");
    }
    label_set = false;
  }
}

void check_configuration(void)
{
  bool changed = 0;
  for(uint8_t i = 0; i < MODULE_64_CHUNKS; i++)
  {
    if(modules_bitmask[i] != modules_bitmask_old[i]) 
    {
      changed = true;
      break;
    }
  }
  //if(changed == true) print_pneumonic_modules();
  //print_pneumonic_modules();
  save_build();
}

static void assign_module_info(uint8_t id, uint8_t module_type, int8_t modules_enum)
{
  uint64_t k = 1;
  modules_bitmask[modules_enum/64] |= (k << (modules_enum%64));
  modules_bitmask[0] |= (k << MOD_SYSTEM);

  modules[module_type].connections[id] = MODULE_CONNECTED;
}

static bool handle_eeprom_id(int8_t *modules_enum, uint8_t *address_to_set, uint8_t module_types_first_address, uint8_t address, uint8_t eeprom_id, uint8_t max_modules_of_given_type, uint8_t module_type)
{
  uint8_t module_update_status = 0;
  //printf("MODULE HANDLER: Address 0x%02X Read ID %d of type 0x%02X\n", address, eeprom_id, module_type);
  
  eeprom_id = 0x00; // added to skip eeprom ID functionality

  // Check if the module has an ID in eeprom
  // If it does check to see if we can add the module at this address
  // If the ID is in conflict then find the closest ID and become assigned
  
  if (eeprom_id < max_modules_of_given_type) 
  {
    if(modules[module_type].connections[eeprom_id] == MODULE_DISCONNECTED)
    {
      *modules_enum = modules_address_to_enum(module_types_first_address + eeprom_id);
      *address_to_set = module_types_first_address + eeprom_id; 
      i2c_set_modules_address(address, 0x00, *address_to_set, *address_to_set);
      assign_module_info(eeprom_id, module_type, *modules_enum);

      i2c_confirm_module_existence_in_scanner(*address_to_set);
      //printf("MODULE HANDLER: Module assigned via eeprom ID at 0x%02X ID %d\n", *address_to_set, eeprom_id);
      return true;
    }
    else 
    {
      //printf("MODULE HANDLER: eeprom_id is in conflict\n");
      for (uint8_t i = 0; i < max_modules_of_given_type; i++)
      {
        if(modules[module_type].connections[i] == MODULE_DISCONNECTED)
        {
          *modules_enum = modules_address_to_enum(module_types_first_address + i);
          *address_to_set = module_types_first_address + i; 
          i2c_set_modules_address(address, 0x00, *address_to_set, *address_to_set);
          set_eeprom_module_id(*address_to_set, i);
          assign_module_info(i, module_type, *modules_enum);

          i2c_confirm_module_existence_in_scanner(*address_to_set);
          //printf("MODULE HANDLER: Module assigned via conflict resolution at 0x%02X, eeprom ID %d\n", *address_to_set, i);

          return true;
        }
      }
    }
  }
  // No eeprom ID assigned
  else
  {
    //printf("MODULE HANDLER: no eeprom read\n");
    for (uint8_t i = 0; i < max_modules_of_given_type; i++)
    {
      if(modules[module_type].connections[i] == MODULE_DISCONNECTED)
      {
        *modules_enum = modules_address_to_enum(module_types_first_address + i);
        *address_to_set = module_types_first_address + i; 
        i2c_set_modules_address(address, 0x00, *address_to_set, *address_to_set);
        set_eeprom_module_id(*address_to_set, i);
        assign_module_info(i, module_type, *modules_enum);

        i2c_confirm_module_existence_in_scanner(*address_to_set);
        //printf("MODULE HANDLER: Module assigned via conflict resolution at 0x%02X, ID %d\n", *address_to_set, i);
        return true;
      }
    }
  }
  return false;
}

void add_module(uint8_t address, uint8_t eeprom_id, uint8_t* buf)
{
  int8_t  modules_enum =           -1;
  uint8_t address_to_set =        0;
  uint8_t firmware_status =       0;
  uint8_t module_types_first_address =  0;
  uint8_t max_modules_of_given_type =   0;
  uint8_t module_type = buf[1];

  if(module_type == MODULE_BOOTLOADER)
  {
    //printf("MODULE HANDLER: adding bootloader at 0x%02x\n", address);
    return;
  }

  max_modules_of_given_type = modules[module_type].max_modules_of_type;
  module_types_first_address = modules[module_type].addresses[0];
  //if(max_modules_of_given_type == 0) printf("MODULE HANDLER: unknown module type at 0x%02x\n", address);

  handle_eeprom_id(&modules_enum, &address_to_set, module_types_first_address, address, eeprom_id, max_modules_of_given_type, module_type);
}

void delete_module(uint8_t address)
{
  uint8_t module_update_status = 0;
  uint8_t modules_enum = modules_address_to_enum(address);
  uint64_t k = 1;
  modules_bitmask[modules_enum/64] &= ~(k << (modules_enum%64));
  modules_bitmask[0] |= (k << MOD_SYSTEM);

  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if(modules[i].addresses[j] == address)
      {
        modules[i].connections[j] = MODULE_DISCONNECTED;
        break;
      }
    }
  }
  //printf("MODULE HANDLER: deleting module at 0x%02x\n", address);
}

void initiate_module_disconnection(uint8_t address)
{
  set_modules_status_from_address(address, MODULE_DISCONNECTING);
}

void module_disconnection_handler(void)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      if (modules[i].connections[j] == MODULE_CONNECTED) modules[i].modules_disconnection_count[j] = 0;
      else if (modules[i].connections[j] == MODULE_DISCONNECTING) modules[i].modules_disconnection_count[j]++;
      if (modules[i].modules_disconnection_count[j] == 5)
      {
        modules[i].modules_disconnection_count[j] = 0;
        modules[i].connections[j] = MODULE_DISCONNECTED;
        delete_module(modules[i].addresses[j]);
      }
    }
  }
}

bool reset_modules_action_or_trigger(uint8_t modules_enum, uint8_t modules_action_or_trigger_number)
{
  if (modules_action_or_trigger_number == MODULES_FIRST_ACTION_OR_TRIGGER)
  {
    set_modules_action_or_trigger_struct(modules_enum, -1, MODULES_FIRST_ACTION_OR_TRIGGER);
  }
  else
  {
    set_modules_action_or_trigger_struct(modules_enum, -1, MODULES_SECOND_ACTION_OR_TRIGGER);
  }
  uint8_t modules_address = modules_enum_to_address(modules_enum);
  if ((modules_address >= MOTOR1_ADD) && (modules_address <= MOTOR6_ADD))
  {
    uint8_t bitmask = 255;
    bitmask &= ~(1UL << (modules_address - MOTOR1_ADD));
    drive_action_status &= bitmask;
  }
  return true;
}

void reset_drive_motors(uint8_t action_id)
{
  for(uint8_t k = 0; k < modules[MODULE_MOTOR].max_modules_of_type; k++)
  {
    if(modules[MODULE_MOTOR].action_trigger_ids[k][0] == action_id)
    {
      set_motor_speed(modules[MODULE_MOTOR].addresses[k], 0); // ensure all motors are stopped with the same id
      reset_modules_action_or_trigger(modules[MODULE_MOTOR].module_enum[k], MODULES_FIRST_ACTION_OR_TRIGGER);
    }
  }
}

bool check_all_actions()
{
  bool action_status = false;
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      //if the module is connected and has an ongoing action or trigger then we check
      if (((modules[i].connections[j] == MODULE_CONNECTED) || (modules[i].connections[j] == MODULE_DISCONNECTING)) && ((modules[i].action_trigger_ids[j][0] != ACTION_NOT_SET) || (modules[i].action_trigger_ids[j][1] != ACTION_NOT_SET)))
      {
        if(modules[i].action_trigger_ids[j][MODULES_FIRST_ACTION_OR_TRIGGER] == ACTION_ID || modules[i].action_trigger_ids[j][MODULES_SECOND_ACTION_OR_TRIGGER] == ACTION_ID) action_status = true;
      }
    }
  }
  return action_status;
}

bool check_module_action_status(uint8_t mod_type, uint8_t mod_id)
{
  bool action_status = false;
  if(module_type_and_id_check(mod_type, mod_id) == false) return false;
  
  if(modules[mod_type].action_trigger_ids[mod_id][MODULES_FIRST_ACTION_OR_TRIGGER] == ACTION_ID || modules[mod_type].action_trigger_ids[mod_id][MODULES_SECOND_ACTION_OR_TRIGGER] == ACTION_ID)
  {
     action_status = true;
  }
  return action_status;
}

bool check_module_trigger_status(uint8_t mod_type, uint8_t mod_id)
{
  bool trigger_status = false;
  if(module_type_and_id_check(mod_type, mod_id) == false) return false;
  
  if(modules[mod_type].action_trigger_ids[mod_id][MODULES_FIRST_ACTION_OR_TRIGGER] == ACTION_ID || modules[mod_type].action_trigger_ids[mod_id][MODULES_SECOND_ACTION_OR_TRIGGER] == ACTION_ID)
  {
     trigger_status = true;
  }
  return trigger_status;
}

void read_module_events(void)
{
  for(uint8_t i = 0; i < NUM_MODULE_TYPES; i++)
  {
    for(uint8_t j = 0; j < modules[i].max_modules_of_type; j++)
    {
      //if the module is connected and has an ongoing action or trigger then we check
      if (((modules[i].connections[j] == MODULE_CONNECTED) || (modules[i].connections[j] == MODULE_DISCONNECTING)) && ((modules[i].action_trigger_ids[j][0] != ACTION_NOT_SET) || (modules[i].action_trigger_ids[j][1] != ACTION_NOT_SET)))
      {
        modules[i].check(modules[i].addresses[j], modules[i].module_enum[j], modules[i].action_trigger_ids[j][0], modules[i].action_trigger_ids[j][1]);
      }
    }
  }
}
