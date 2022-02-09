#include "i2c_bus.h"
#include "modules.h"
#include "robowunderkind.h"

void scan()
{
  for(uint8_t i = 0; i < 6; i ++)
  {
    i2c_scan();
    check_configuration();
    module_disconnection_handler();
    vTaskDelay(75 / portTICK_PERIOD_MS);
  }
}

/*----------------------------------------------------------- System -----------------------------------------------------------------*/
    _System::_System()
    {
      
    }


/*========================================================= ACTUATORS =======================================================================================*/

/*----------------------------------------------------------- Motor -----------------------------------------------------------------*/    
  
    
    _Motor::_Motor()
    {
      
    }

    bool _Motor::torque(uint8_t module_index, int8_t torque_pct)
    {
      bool result = false;
      result = set_motor_pwm(module_index, torque_pct); 
      if(!result) scan();
      return result;
    }
  
    bool _Motor::speed(uint8_t module_index, int8_t speed_pct)
    {
      bool result = false;
      result = set_motor_pwm(module_index, speed_pct); 
      if(!result) scan();
      return result;
    }
  
    bool _Motor::stop(uint8_t module_index)
    {
      bool result = false;
      result = set_motor_pwm(module_index, 0); 
      if(!result) scan();
      return result;
    }
  
    bool _Motor::stop_all()
    {
      bool result = false;
      for(uint8_t  i = 0; i < MAX_MOTORS; i++)
      {
        if(check_if_attached_from_index(MOTOR, i))
        {
          result = set_motor_pwm(i, 0); 
          if(!result) scan();
        }
      }
      return result;
    }
  
    bool _Motor::drive(uint8_t module_index, int8_t speed_pct, uint16_t distance_cm, uint16_t wheeldiameter)
    {
      bool result = false;
      uint8_t wheel_diameterHSB = wheeldiameter >> 8;
      uint8_t wheel_diameterLSB = wheeldiameter >> 0;
      uint16_t distance_mm = distance_cm*10;
      uint8_t distanceHSB = distance_mm >> 8;
      uint8_t distanceLSB = distance_mm >> 0;
  
      if(distance_cm <= 0 || speed_pct == 0) return false;
  
      set_module_action_trigger_id(MOTOR, module_index, ACTION_ID, MODULES_FIRST_ACTION_OR_TRIGGER);
      result = set_motor_action(module_index, speed_pct, wheel_diameterHSB, wheel_diameterLSB, distanceHSB, distanceLSB);
      printf("Drive Speed = %d Drive Distance = %d\n", speed_pct, distance_cm);
      if(!result) scan();
      return result;
    }
  
    bool _Motor::set_drives(int8_t speed_pct_L, int8_t speed_pct_R, uint16_t distance_L, uint16_t distance_R, uint16_t wheelD)
    {
      bool result = false;
      bool result_final = true;
      for (uint8_t motor_number = 0; motor_number < MAX_MOTORS; motor_number++)
      {
        int i = 0;
        if (this->_config_bitmask & (i |= 1 << motor_number)) 
        {
          if (this->_directions_bitmask & (i |= 1 << motor_number))
          {
            if (distance_R >= MAX_DISTANCE) result = this->speed(motor_number, speed_pct_R);
            else     result = this->drive(motor_number, speed_pct_R, distance_R, wheelD);
          }
          else
          { 
            if (distance_L >= MAX_DISTANCE) result = this->speed(motor_number, speed_pct_L);
            else     result = this->drive(motor_number, speed_pct_L, distance_L, wheelD);
          }
        }
        if(!result) result_final = false;
      }
      if(!result_final) scan();
      return result_final;
    }
  
    bool _Motor::forward(int8_t speed_pct, uint16_t distance_cm, uint16_t wheeldiameter)
    {
      bool result = false;
      if(distance_cm <= 0 || speed_pct == 0) return false;
      result = this->set_drives(speed_pct, -speed_pct, distance_cm, distance_cm);
      return result;
    }
  
    bool _Motor::backward(int8_t speed_pct, uint16_t distance_cm, uint16_t wheeldiameter)
    {
      bool result = false;
      if(distance_cm <= 0 || speed_pct == 0) return false;
      result = this->set_drives(-speed_pct, speed_pct, distance_cm, distance_cm);
      return result;
    }
  
    bool _Motor::turn(int8_t speed_pct, float angle, uint16_t turning_diameter, uint16_t wheeldiameter)
    {
      bool result = false;
      int8_t speed_L, speed_R;
      if(speed_pct == 0) return false;
      uint16_t distance = abs(ceil((turning_diameter)*PI)*(angle/360.0));
      if(angle < 0) result = this->set_drives(-speed_pct, -speed_pct, distance, distance);
      else result = this->set_drives(speed_pct, speed_pct, distance, distance);
      return result;
    }
  
    void _Motor::config(bool *motors, bool *directions)
    {
      this->_config_bitmask = 0;
      this->_directions_bitmask = 0;
      for(uint8_t i = 0; i < MAX_MOTORS; i++)
      {
        if(motors[i]) this->_config_bitmask |= 1 << i;
        if(directions[i]) this->_directions_bitmask |= 1 << i;
      }
    }

/*----------------------------------------------------------- Servo -----------------------------------------------------------------*/

    _ServoMotor::_ServoMotor()
    {
      
    }

/*----------------------------------------------------------- Claw -----------------------------------------------------------------*/

    _Claw::_Claw()
    {
      
    }

/*----------------------------------------------------------- Hinge -----------------------------------------------------------------*/

    _Hinge::_Hinge()
    {
      
    }

/*----------------------------------------------------------- Display -----------------------------------------------------------------*/

    _Display::_Display()
    {
      
    }

    bool _Display::text(uint8_t module_index, uint8_t orientation, char* text, uint16_t scrolling_rate_ms)
    {
      uint8_t total_length = strlen(text) + 1;
      if(orientation > 3 || module_index > MAX_DISPLAYS - 1 || total_length <= 1) return false;
      uint8_t starting_index = 0;
      char chars[16] = {0}; 
      uint8_t last_char = 0;
      uint8_t index = 0;
      bool result = false;
  
      uint8_t rateHSB = scrolling_rate_ms >> 8;
      uint8_t rateLSB = scrolling_rate_ms >> 0;
      
      for(uint8_t c = 0; c < total_length; c++)
      {
        chars[index] = text[c];
        index++;
        if(c % 15 == 0 && c != 0)
        {
          load_custom_text(module_index, chars, last_char, 16);
          last_char += c;
          index = 0;
          memset(chars, 0, 16);
        }
      }
      uint8_t l = strlen(chars);
      if(l != 0) load_custom_text(module_index, chars, last_char, l);
  
      set_module_action_trigger_id(DISPLAY, module_index, ACTION_ID, MODULES_FIRST_ACTION_OR_TRIGGER);
      result = set_custom_display_text(module_index, orientation, total_length, rateHSB, rateLSB);
      if(!result) scan();
      return result;
    }
  
    bool _Display::custom_image(uint8_t module_index, const uint8_t* image, uint8_t orientation, uint16_t time_ms)
    {
      bool result = false;
      uint8_t rows[16];
      uint8_t delayHSB = time_ms >> 8;
      uint8_t delayLSB = time_ms >> 0;
  
      if(sizeof(image) != 4) return false;
  
      for(uint8_t i = 0; i < 16; i++)
      {
        rows[i] = image[i];
      }
      load_custom_display_image(module_index, 1, rows);
      for(uint8_t j = 0; j < 16; j++)
      {
        rows[j] = image[j+16];
      }
      load_custom_display_image(module_index, 2, rows);
      set_module_action_trigger_id(DISPLAY, module_index, ACTION_ID, MODULES_FIRST_ACTION_OR_TRIGGER);
      result = set_display_image(module_index, 0xFF, orientation, delayHSB, delayLSB);
      if(!result) scan();
      return result;
    }
  
    bool _Display::animation(uint8_t module_index, uint8_t orientation, uint8_t animation_num, uint16_t frame_rate)
    {
      bool result = false;
      uint8_t frame_rateHSB = frame_rate >> 8;
      uint8_t frame_rateLSB = frame_rate >> 0;
  
      set_module_action_trigger_id(DISPLAY, module_index, ACTION_ID, MODULES_FIRST_ACTION_OR_TRIGGER);
      result = set_animation(module_index, animation_num, 1, 1, orientation, 0xFF, frame_rateHSB, frame_rateLSB); 
      if(!result) scan();
      return result;
    }
  
    bool _Display::custom_animation(uint8_t module_index, const uint8_t (*frames)[32], uint8_t num_frames, uint8_t orientation, uint16_t frame_rate)
    {
      bool result = false;
      uint8_t rows[16] = {0};
      uint8_t frame_rateHSB = frame_rate >> 8;
      uint8_t frame_rateLSB = frame_rate >> 0;
  
      //printf("Number of frames = %d\n", num_frames);
      if(num_frames > 5) return result;
      
      for(uint8_t i = 0; i < num_frames; i++)
      {
        for(uint8_t j = 0; j < 16; j++)
        {
          rows[j] = frames[i][j];
        }     
        load_custom_display_animation(module_index, i, 1, rows);
        //printf("Loading Animation Frame %d first half\n", i);
        memset(rows, 0, 16);
        for(uint8_t k = 0; k < 16; k++)
        {
          rows[k] = frames[i][k+16];
        }     
        load_custom_display_animation(module_index, i, 2, rows);
        //printf("Loading Animation Frame %d second half\n", i);
        memset(rows, 0, 16);
      }
      set_module_action_trigger_id(DISPLAY, module_index, ACTION_ID, MODULES_FIRST_ACTION_OR_TRIGGER);
      result = set_animation(module_index, 0xff, 1, 0, orientation, 0xFF, frame_rateHSB, frame_rateLSB);
      if(!result) scan(); 
      return result;
    }

/*------------------------------------------------------------ LED -----------------------------------------------------------------*/

    _LED::_LED()
    {
      
    }

    bool _LED::blink(uint8_t module_index, uint8_t r, uint8_t g, uint8_t b, uint8_t blinks, float frequency)
    {
      bool result = false;
      if(frequency == 0) frequency = 1;
      float period = 1000.0/((float)frequency);
      uint8_t timeHSB = (uint16_t)period >> 8;
      uint8_t timeLSB = (uint8_t)period;
  
      set_module_action_trigger_id(RGB_LED, module_index, 1, MODULES_FIRST_ACTION_OR_TRIGGER);
      result = set_rgb_action(module_index, r, g, b, timeHSB, timeLSB, blinks ,0);
      if(!result)
      {
        scan();
      }
      return result;
    }
  
    bool _LED::rgb(uint8_t module_index, uint8_t r, uint8_t g, uint8_t b)
    {
      bool result = false;
      result = set_rgb_color(module_index, r, g, b);
      if(!result) scan();
      return result;
    }

/*========================================================= SENSORS =======================================================================================*/
    
/*-------------------------------------------------- Ultrasonic Distance Sensor ---------------------------------------------------------------------------*/

    _Ultrasonic::_Ultrasonic()
    {
      
    }

    uint16_t _Ultrasonic::read(uint8_t module_index)
    {
      float sensor_value;
      bool result = 0;
      result = read_ultrasonic_distance(module_index, &sensor_value);
      if(!result) scan();
      return sensor_value;
    }

/*----------------------------------------------------------- IMU -----------------------------------------------------------------*/

    _IMU::_IMU()
    {
      
    }

    bool _IMU::read_gyroscope(uint8_t module_index, float *gyrox, float *gyroy, float *gyroz)
    {
      bool result;
      result = read_gyro(module_index, gyrox, gyroy, gyroz);
      if(!result) scan();
      return result;
    }
  
    bool _IMU::read_accelerometer(uint8_t module_index, float *accx, float *accy, float *accz)
    {
      bool result;
      result = read_accelerometer_values(module_index, accx, accy, accz);
      if(!result) scan();
      return result;
    }

/*----------------------------------------------------------- Button -----------------------------------------------------------------*/    

    _Button::_Button()
    {
      
    }

    uint8_t _Button::read(uint8_t module_index)
    {
      uint8_t state = 0;
      bool result = read_button_state(module_index, &state);
      if(!result) scan();
      return state;
    }


/*-------------------------------------------------------- Line Tracker -----------------------------------------------------------------*/

    _LineTracker::_LineTracker()
    {
      
    }

    bool _LineTracker::read_all(uint8_t module_index, linetracker_sensorvalues *values)
    {
      read_linetracker_sensorvalues(module_index, values, LINETRACKER_SATURATION );
    }

/*-------------------------------------------------------- Light Sensor -----------------------------------------------------------------*/

    _Light::_Light()
    {
      
    }

    uint16_t _Light::read(uint8_t module_index)
    {
      float sensor_value;
      bool result = 0;
      result = read_lightsensor(module_index, &sensor_value);
      if(!result) scan();
      return sensor_value;
    }

/*-------------------------------------------------------- Motion Sensor -----------------------------------------------------------------*/

    _Motion::_Motion()
    {
      
    }

    uint8_t _Motion::read(uint8_t module_index)
    {
      uint8_t state = 0;
      bool result = read_pir_state(module_index, &state);
      if(!result) scan();
      return state;
    }
  
