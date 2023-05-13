// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Voltage Generator
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include "Class_VoltageGenerator.h"



// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// * * * * * * * * * * * * * * * * * * *
//    Without Pin

VoltageGenerator::VoltageGenerator() {
  
  // set pin to output GND
  pinMode(_pin, OUTPUT);
  analogWrite(_pin, 0);
  

};


// * * * * * * * * * * * * * * * * * * *
//    With Pin

VoltageGenerator::VoltageGenerator(int pin) {
  
  // store given pin number
  _pin = pin;
  
  // set pin to output GND
  pinMode(_pin, OUTPUT);
  analogWrite(_pin, 0);
  
  
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set State
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void VoltageGenerator::setState(bool state) {
  
  if (state) {
    analogWrite(_pin, 255);
  } else {
    analogWrite(_pin, 0);
  };
}


// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Voltage
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void VoltageGenerator::setVoltage(float volt) {
  
  // limit range
  volt = min(max(volt, 0), 5);
  
  // expand to 0-255
  volt *= 51;
  
  // write signal
  analogWrite(_pin, volt);
};