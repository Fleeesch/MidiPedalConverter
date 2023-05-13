// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : MIDI
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include "Class_Midi.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Static Initialization
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// port index reset
int Midi::port_index = 0;

// declare ports lookup array
MidiPort *Midi::ports[Midi::MIDI_PORT_MAX];

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Static Method : Add Port
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void Midi::addPort(MidiPort *port)
{

  // stay within port limits
  if (Midi::port_index >= MIDI_PORT_MAX)
    return;

  // store midi port
  Midi::ports[Midi::port_index] = port;

  // increment port index
  Midi::port_index++;
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Static Method : Send to all Ports
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void Midi::sendToAllPorts(int b1, int b2, int b3)
{

  // go through ports
  for (MidiPort *port : Midi::ports)
  {

    // stop when there are no more ports to process
    if (port == NULL)
      return;

    // send message to all ports
    port->sendMessage(b1, b2, b3);
  }
}