#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_MidiHandler.h"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : MIDI Handler : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_MidiHandler_h
#define Class_MidiHandler_h

#include <Arduino.h>
#include "MidiPedalConverter.h"
#include "Class_MidiPort.h"

class MidiPort;

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Class
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class MidiHandler
{

public:
  // maximum count of midi ports
  const static int MIDI_PORT_MAX = 10;
  
  // basic constructor
  MidiHandler();
  
  // list of MIDI ports
  static MidiPort *ports[];
  
  // add a referenced MIDI port to list
  static void addPort(MidiPort *port);
  
  // send a message to all ports output
  static void sendToAllPorts(int b1, int b2, int b3 = -1);
  
  // send a message to all ports output
  static void sendSysExToAllPorts(int bytes[], size_t size);

private:
  // current port index (for port creation)
  static int port_index;
};

#endif