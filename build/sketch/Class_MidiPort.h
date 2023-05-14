#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_MidiPort.h"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : MIDI Port : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_MidiPort_h
#define Class_MidiPort_h

#include <Arduino.h>
#include "Class_MidiHandler.h"
#include "MidiPedalConverter.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Class
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class MidiPort
{

public:
  MidiPort();

  // send message to port output
  virtual void sendMessage(int b1, int b2, int b3 = -1);

  virtual void sendByte(int b);

private:
};

#endif