#include "robowunderkind.h"
#include "custom_audio.h"


// Make your Text to Speech MP3 here https://ttsmp3.com/, or find any audio file you like
// Convert MP3 to mono channel wav file 8 bit 11025 Hz here https://audio.online-convert.com/convert-to-wav
// Open the wav file in Audacity https://www.audacityteam.org/download/
// Export Audio... Header RAW, Unsigned 8-bit PCM
// Open the exported file in HxD https://download.cnet.com/HxD-Hex-Editor/3000-2352_4-10891068.html
// Select all, edit copy as C then paste into custom_audio.h as const uint8_t

RoboWunderkind RW = RoboWunderkind();
bool state = false;

void setup() 
{
  Serial.begin(115200);
  RW.begin();  
}

void loop() 
{  
  if(RW.Button.read(0) == true) 
  {
    state = !state;
    if(state == true) 
    {
      RW.LED.rgb(0, 255, 255, 255);
      RW.audio(audiosample_lights_on, sizeof(audiosample_lights_on), 44100);
    }
    else
    {
      RW.LED.rgb(0, 0, 0, 0);
      RW.audio(audiosample_lights_off, sizeof(audiosample_lights_off), 44100);
    }
  }
}
