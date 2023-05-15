#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_Settings.h"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Settings : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_Settings_h
#define Class_Settings_h

#include <Arduino.h>
#include "MidiPedalConverter.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class Settings
{

public:
  settings();
  
  // use 14 Bit
  static bool MIDI_USE_14BIT;
  
  // interpolation value for interpolated signals
  static float MIDI_INTERPOLATION;
  
  // initialize settings
  static void init();
  
  // update settings
  static void update();
  
  // last interpolation value
  static int M_INT_LAST;
  
  // freeze state for interpolation setting reading
  static bool M_INT_FREEZE;
  
  // value point when freeze is activated
  static int M_INT_FREEZE_PNT;
  
  // interpolation debounce ticks required for a proper reading
  static const int M_INT_CHANGE_THRESH = 2;
};

#endif