// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Pedal : Expression : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_PedalExpression_h
#define Class_PedalExpression_h

#include <Arduino.h>
#include "MidiPedalConverter.h"
#include "Class_PedalInterface.h"
#include "Class_Pedal.h"
#include "Class_Midi.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class PedalExpression : public Pedal
{

public:
  // constructor with pedal interface reference
  PedalExpression(PedalInterface *pedal_if);
  
  // routine, called indirectly by main loop
  void routine();
  
  // get pedal value
  int getValue();
  
  // midi send functions
  void sendMidi(int value);
  void sendMidi14Bit(int lsb, int msb);
  void sendMidi7Bit(int value);
  
  // get control change
  virtual int getControlChange()
  {
    return 11;
  };
  // get control change LSB
  virtual int getControlChangeLSB()
  {
    return 43;
  };

private:
  // interpolation factor (0.0 - 1.0, smaller = slower)
  float _interpolation_factor = 0.1;
  
  int _value_min = 110;
  int _value_max = 1015;
  
  // last sent MIDI value
  int _out_last = 0;
  
  // last calculated value (float for the purpose of slow interpolation)
  float _value_last = 0;
  
  // state of timeout (true = hysteresis active)
  bool _timeout = true;

  // last value during timeout trigger
  int _timeout_val = 0;

  // ticks for the timeout to turn on
  int _timeout_ticks = 5000;

  // timeout counter
  int _timeout_counter = 0;

  // timeout threshold (value difference to deactivate timeout)
  int _timeout_threshold = 4;

  // last midi value
  int last_midi_value = 0;
  int last_midi_value_lsb = 0;
};

#endif