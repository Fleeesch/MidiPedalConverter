// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Pedal : Expression : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_PedalExpression_h
#define Class_PedalExpression_h

#include <Arduino.h>
#include "Class_Settings.h"
#include "MidiPedalConverter.h"
#include "Class_PedalInterface.h"
#include "Class_Pedal.h"
#include "Class_MidiHandler.h"

#define EXPRESSION_PEDAL_LOW 50
#define EXPRESSION_PEDAL_HIGH 1000

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
  
  // reset values
  void reset();
  
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
  
  // tries to escape freeze, true on success
  bool _escapeFreeze();
  
  // calculates delta, true on freeze trigger
  bool _applyFreeze();
  
  // interpolation factor (0.0 - 1.0, smaller = slower)
  float _interpolation_factor = 0.1;
  
  int _value_min;
  int _value_max;
  
  // last sent MIDI value
  int _out_last = 0;
  
  // last calculated value (float for the purpose of slow interpolation)
  float _value_last = 0;
  
  // target value
  int _value_target = 0;
  
  // last target value, for delta calculation
  int _target_last = 0;
  
  // last midi value
  int _last_midi_value_msb = 0;
  int _last_midi_value_lsb = 0;
  
  // value delta
  int _value_delta = 0;
  
  // value delta threshold for causing a freeze (smaller > slower reaction)
  const int _VALUE_DELTA_THRESHOLD = 40;
  
  // maximum delta value (limits excessive freeze blocking)
  const int _VALUE_DELTA_CAP = 1000;
  
  // freeze = true -> do nothing until value escapes feeze point
  bool _freeze = false;
  
  // freez point value, gets stored during freeze trigger
  int _freeze_value = 0;
  
  // threshold for freeze escape in analog value distance
  const int _FREEZE_THRESHOLD = 3;
  
  // last sent message microsecond timestamp
  unsigned long _last_sent_micros = 0;

};

#endif