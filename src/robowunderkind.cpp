/*
 * robowunderkind.cpp
 *
 */

#include "robowunderkind.h"

//=========================================================GPIO INIT==========================================================================================

void init_gpios(void)
{
  pinMode(GPIO_BOOST_IC_EN, OUTPUT);
  pinMode(GPIO_BOOST_EN, OUTPUT);   
  pinMode(GPIO_AUDIO_EN, OUTPUT);
  pinMode(GPIO_ONOFF, INPUT);
  pinMode(GPIO_CHARGE_SENSE, INPUT);
  pinMode(GPIO_USB_DET, INPUT);
  pinMode(GPIO_BATTERY_SENSE, INPUT);
  pinMode(0, INPUT_PULLUP);

  digitalWrite(GPIO_AUDIO_EN, LOW);
}

//================================================================PM FUNCTIONS==========================================================================

uint8_t battery_voltage_to_percentage(uint16_t voltage)
{
  if    (voltage < 3630)              return 0;
  else if ((voltage >= 3630) && (voltage < 3700))   return 10;
  else if ((voltage >= 3700) && (voltage < 3750))   return 20;
  else if ((voltage >= 3750) && (voltage < 3820))   return 30;
  else if ((voltage >= 3820) && (voltage < 3880))   return 40;
  else if ((voltage >= 3880) && (voltage < 3920))   return 50;
  else if ((voltage >= 3920) && (voltage < 3970))   return 60;
  else if ((voltage >= 3970) && (voltage < 4020))   return 70;
  else if ((voltage >= 4020) && (voltage < 4100))   return 80;
  else if ((voltage >= 4100) && (voltage < 4140))   return 90;
  else                          return 100;
}

uint8_t get_battery_level(void)
{
  float average_adc_value = 0;
  float   battery_voltage = 0;
  uint8_t battery_percentage = 0;

  for (uint8_t i = 0; i < NO_OF_SAMPLES; i++)
  {
    uint16_t raw_adc_value = analogRead(GPIO_BATTERY_SENSE);
    average_adc_value += raw_adc_value;
  }
  average_adc_value /= NO_OF_SAMPLES;

  battery_voltage = (average_adc_value/4096.0)*3300 * BATTERY_CONVERSION_RATIO;
  battery_percentage = battery_voltage_to_percentage((uint16_t)battery_voltage);
  //printf("PM: raw adc is %f, voltage is %f, battery is at %d percent\n", average_adc_value, battery_voltage, battery_percentage);

  return battery_percentage;
}

bool get_charger_onoff_state()
{
  bool charger_on_off = digitalRead(GPIO_CHARGE_SENSE);
  return !charger_on_off;
}

uint8_t get_usb_state(void)
{
  uint8_t usb_state = digitalRead(GPIO_USB_DET);
  //printf("USB State = %d \n", usb_state);
  if     (usb_state == 0) return USB_UNPLUGGED;
  else if(usb_state == 1 && get_charger_onoff_state() == 0) return USB_CHARGING_DONE;
  else if(usb_state == 1 && get_charger_onoff_state() == 1) return USB_PLUGGED;
  else return USB_UNPLUGGED;
}

uint8_t get_battery_state (void)
{
  if    (get_charger_onoff_state() == 1 && get_usb_state() == 1) return BATTERY_CHARGING;
  else if (get_charger_onoff_state() == 0 && get_usb_state() == 1) return BATTERY_FULL;
  else  return BATTERY_DISCHARGING;
}

void set_boost_converter(bool on_off)
{
 digitalWrite(GPIO_BOOST_IC_EN, on_off);
 digitalWrite(GPIO_BOOST_EN, on_off);
 vTaskDelay(50 / portTICK_PERIOD_MS);
}

void print_heap()
{
  size_t heap_size = 0;
  heap_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  printf("MAIN: Heap Size = %d\n", heap_size);
}

//========================================================= INIT ====================================================================================

void init_robo()
{
  init_gpios();
  init_i2c();
  init_modules();
  heap_caps_check_integrity_all(true);
  
}

esp_err_t get_mac_address(uint8_t *mac)
{
  esp_err_t ret = ESP_OK;
  ret = esp_efuse_mac_get_default(mac);
  /*
  if(ret == ESP_OK) 
  {
    printf("MAIN: MAC Address is %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  else
  {
    printf("Error Reading MAC Address\n");
  }
  */
  return ret;
}

//========================================================= ROBO WUNDERKIND CLASS ====================================================================================
  RoboWunderkind::RoboWunderkind()
  {
    init_robo();
    this->mac_address();
    
    // Ensure we hard reset the modules
    this->enable_power(BOOST_CONVERTER_OFF);
    this->enable_power(BOOST_CONVERTER_ON);
    
    //vTaskDelay(250 / portTICK_PERIOD_MS);
    //this->scan();
  }

  void RoboWunderkind::print_attached_modules()
  {
    print_pneumonic_modules();
  }

  void RoboWunderkind::wait_for_all_actions()
  {
    while(check_all_actions())
    {
      vTaskDelay(50 / portTICK_PERIOD_MS);
      read_module_events();
    }
  }

  void RoboWunderkind::wait_for_action(uint8_t mod_type, uint8_t mod_id)
  {
    while(check_module_action_status(mod_type, mod_id))
    {
      vTaskDelay(50 / portTICK_PERIOD_MS);
      read_module_events();
    }
  }

  bool RoboWunderkind::is_attached(uint8_t mod_type, uint8_t id)
  {
    return check_if_attached_from_index(mod_type, id);
  }

  void RoboWunderkind::enable_power(bool on_off)
  {
    set_boost_converter(on_off);
    this->_boost_converter_state = on_off;
  }

  void RoboWunderkind::mac_address()
  {
    get_mac_address(this->_mac_addr);
  }
  
