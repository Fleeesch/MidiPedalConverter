// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Voltage Generator : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_VoltageGenerator_h
#define Class_VoltageGenerator_h

#include <Arduino.h>
#include "MidiPedalConverter.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Class
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class VoltageGenerator {

public:
  
  
  VoltageGenerator();
  
  // constructor with voltage out pin
  VoltageGenerator(int pin);
  
  // set variable voltage via PWM signal
  void setVoltage(float volt);
  
  // set binary voltage level (0V, 5V)
  void setState(bool state);

private:
  
  // Voltage Out Pin
  int _pin = 3;
};

#endif