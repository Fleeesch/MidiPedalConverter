#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_MidiPortUsb.h"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : MIDI Port : USB : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_MidiPortUsb_h
#define Class_MidiPortUsb_h

#include "MidiPedalConverter.h"
#include <Midi.h>
#include "Class_MidiPort.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Class
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class MidiPortUsb : public MidiPort
{

public:
  MidiPortUsb();
  
  // send message to serial port
  void sendMessage(int b1, int b2, int b3 = -1);
  
  void sendByte(int b);

private:
  
};

#endif