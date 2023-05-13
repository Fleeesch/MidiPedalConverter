// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Pedal : Sustain : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_PedalSustain_h
#define Class_PedalSustain_h

#include <Arduino.h>
#include "MidiPedalConverter.h"
#include "Class_Pedal.h"
#include "Class_Midi.h"

const int SUSTAIN_DEBOUNCE_TICKS = 20;

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class PedalSustain : public Pedal {

public:

  // constructor with reference to pedal interface
  PedalSustain(PedalInterface* pedal_if);

  // routine, called indirectly by main loop
  void routine();
  
  // get control change number
  int getControlChange() {
    return 64;
  };

private:

  // current state (false = release, true = pressed)
  bool state = false;

  // debounce counter
  int _signal_debounce = 0;
};

#endif