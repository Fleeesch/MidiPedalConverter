#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_Pedal.h"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Pedal : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_Pedal_h
#define Class_Pedal_h

#include <Arduino.h>
#include "MidiPedalConverter.h"
#include "Class_PedalInterface.h"

class PedalInterface;

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class Pedal
{

public:
  Pedal();

  // constructor with pedal interface reference
  Pedal(PedalInterface *p_interface);

  // linked pedal interface
  PedalInterface *pedal_interface;

  // routine, called indirectly by main loop
  virtual void routine();

  // reset values
  virtual void reset();

  // get midi status message
  virtual int getStatusMessage();

  // control change number getter
  virtual int getControlChange() { return 0; };
  virtual int getControlChangeLSB() { return 32; };

  virtual bool midiIsGo();

  // ignore midi messages on reset
  const int midi_message_dismiss_time = SETTING_PEDAL_MIDI_DELAY;
  long init_millis = 0;

  // midi is allowed to run
  bool midi_is_go = false;

private:
};

#endif