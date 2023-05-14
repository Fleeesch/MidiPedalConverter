#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_MidiPortSerial.h"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : MIDI Port : Serial : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_MidiPortSerial_h
#define Class_MidiPortSerial_h

#include <Arduino.h>
#include "HardwareSerial.h"
#include "MidiPedalConverter.h"
#include "Class_MidiPort.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Class
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class MidiPortSerial : public MidiPort
{

public:
  MidiPortSerial();

  // constructor with serial port reference
  MidiPortSerial(Stream &serialport);

  // send message to serial port
  void sendMessage(int b1, int b2, int b3 = -1);

  void sendByte(int b);

private:
  // serial port reference, use port 1 by default
  Stream &_serialport = Serial;
};

#endif