// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : MIDI Handler
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Static Initialization
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// port index reset
int MidiHandler::port_index = 0;

// declare ports lookup array
MidiPort *MidiHandler::ports[MidiHandler::MIDI_PORT_MAX];

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Static Method : Add Port
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void MidiHandler::addPort(MidiPort *port)
{
  
  
  // stay within port limits
  if (MidiHandler::port_index >= MIDI_PORT_MAX)
    return;
  
  // store midi port
  MidiHandler::ports[MidiHandler::port_index] = port;
  
  // increment port index
  MidiHandler::port_index++;
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Static Method : Send to all Ports
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void MidiHandler::sendToAllPorts(int b1, int b2, int b3)
{
  
  // go through ports
  for (MidiPort *port : MidiHandler::ports)
  {
    
    // stop when there are no more ports to process
    if (port == NULL)
      return;
    
    // send message to all ports
    port->sendMessage(b1, b2, b3);
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Static Method : Send to all Ports
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void MidiHandler::sendSysExToAllPorts(int bytes[], size_t size)
{
  
  // go through ports
  for (MidiPort *port : MidiHandler::ports)
  {
    // stop when there are no more ports to process
    if (port == NULL)
      return;
    
    // opening byte
    port->sendByte(0xF0);
    
    // header
    port->sendByte(0x03E);
    
    for (int i = 0; i < size; i++)
    {
      port->sendByte(bytes[i]);
    }
    
    // closing byte
    port->sendByte(0xF7);
  }
}