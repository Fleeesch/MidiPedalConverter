// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Audio Jack : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_AudioJack_h
#define Class_AudioJack_h

#include <Arduino.h>
#include "MidiPedalConverter.h"


// pin configuration
#define PINMODE_5V 0
#define PINMODE_GND 1
#define PINMODE_DIGITAL 2
#define PINMODE_PULLUP 3
#define PINMODE_ANALOG 4

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Class
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class AudioJack
{

public:
  AudioJack();

  int main(){};

  // constructor covering all pins
  AudioJack(int pdt, int pdr, int pds, int pt, int pr, int ps);

  // pin configuration
  void setupDetection();
  void setupSustain();
  void setupExpression();
  void setupControlVoltage();
  void setupSustainTest();
  void setupDead();

  // signal reading
  bool getDectectionSignal();
  bool getSustainSignal();
  int getExpressionSignal();
  int getControlVoltageSignal();
  bool getSustainTestSignal();
  bool testForControlVoltage();
  
  // pin mode
  void setPinMode(int pin, int mode);

  // pin set
  void setPinSignal(int pin, int signal);

private:
  // detection pins
  int _pin_detect_tip;
  int _pin_detect_ring;
  int _pin_detect_sleeve;

  // jack pins
  int _pin_tip;
  int _pin_ring;
  int _pin_sleeve;
};

#endif