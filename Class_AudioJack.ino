// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Audio Jack
//
//  Represents a TRS Jack
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include "Class_AudioJack.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

AudioJack::AudioJack(){};

AudioJack::AudioJack(int pdt, int pdr, int pds, int pt, int pr, int ps)
{

  // detection pins
  _pin_detect_tip = pdt;
  _pin_detect_ring = pdr;
  _pin_detect_sleeve = pds;

  // jack pins
  _pin_tip = pt;
  _pin_ring = pr;
  _pin_sleeve = ps;

  // setup detection pins
  setPinMode(_pin_detect_tip, PINMODE_PULLUP);
  setPinMode(_pin_detect_ring, PINMODE_PULLUP);
  setPinMode(_pin_detect_sleeve, PINMODE_PULLUP);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Get Detection Signal
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool AudioJack::getDectectionSignal()
{
  return digitalRead(_pin_detect_tip);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Get Sustain Signal
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool AudioJack::getSustainSignal()
{
  return !digitalRead(_pin_tip);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Get Expression Signal
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

int AudioJack::getExpressionSignal()
{
  return analogRead(_pin_tip);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Get Control Voltage Signal
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

int AudioJack::getControlVoltageSignal()
{
  return analogRead(_pin_tip);
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Test for Control Voltage
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool AudioJack::testForControlVoltage()
{

  // predeclare last analog read for initial comparison
  int a_read = 1023;
  int a_read_last;

  // test multiple times...
  for (int i = 0; i < 100; i++)
  {

    // get even / uneven iteration index state
    int mod_val = i % 2;
    
    // toggle voltage between high and low
    VOLTAGE_GENERATOR->setState(mod_val);
    
    // store last analog signal
    a_read_last = a_read;
    
    // store current analog signal
    a_read = analogRead(_pin_tip);
    
    // voltage is 0V and signal has fallen? Everything ok, skip...
    if (mod_val == 0 && a_read_last > a_read)
      continue;
    
    // voltage is 5V and signal has risen? Everything ok, skip...
    if (mod_val == 1 && a_read_last < a_read)
      continue;
    
    // voltage stayed the same, return test as failed
    return false;
  }
  
  // activate voltage generator once again
  VOLTAGE_GENERATOR->setState(1);
  
  // return test as a success
  return true;
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Get Mono Test Signal
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool AudioJack::getSustainTestSignal()
{
  int thresh = 10;
  
  int min_val = 1023;
  int max_val = 0;
  
  for (int i = 0; i < 100; i++)
  {
    
    int a_read = analogRead(_pin_tip);
    
    min_val = min(min_val, a_read);
    max_val = max(max_val, a_read);
  }
  
  return (max_val - min_val) > thresh;
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Setup Detection
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setupDetection()
{
  
  setPinMode(_pin_tip, PINMODE_GND);
  setPinMode(_pin_ring, PINMODE_GND);
  setPinMode(_pin_sleeve, PINMODE_GND);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Setup Sustain
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setupSustain()
{
  
  setPinMode(_pin_tip, PINMODE_PULLUP);
  setPinMode(_pin_ring, PINMODE_GND);
  setPinMode(_pin_sleeve, PINMODE_GND);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Setup Expression
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setupExpression()
{

  setPinMode(_pin_tip, PINMODE_ANALOG);
  setPinMode(_pin_ring, PINMODE_5V);
  setPinMode(_pin_sleeve, PINMODE_GND);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Setup Control Voltage
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setupControlVoltage()
{

  setPinMode(_pin_tip, PINMODE_ANALOG);
  setPinMode(_pin_ring, PINMODE_GND);
  setPinMode(_pin_sleeve, PINMODE_GND);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Setup Mono Test
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setupSustainTest()
{

  setPinMode(_pin_tip, PINMODE_ANALOG);
  setPinMode(_pin_ring, PINMODE_5V);
  setPinMode(_pin_sleeve, PINMODE_GND);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Setup Dead
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setupDead()
{

  setPinMode(_pin_tip, PINMODE_ANALOG);
  setPinMode(_pin_ring, PINMODE_GND);
  setPinMode(_pin_sleeve, PINMODE_GND);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Pin Signal
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setPinSignal(int pin, int level)
{

  // skip if pin isn't valid
  if (!pin)
  {
    return;
  }

  // change pin level
  digitalWrite(pin, level);
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Pin Mode
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setPinMode(int pin, int mode)
{
  // skip if pin isn't valid
  if (!pin)
  {
    return;
  }

  // change pin configuration depending on layout
  switch (mode)
  {
  case PINMODE_DIGITAL:
    pinMode(pin, INPUT);
    digitalWrite(pin, LOW);

  case PINMODE_PULLUP:
    pinMode(pin, INPUT_PULLUP);
    digitalWrite(pin, HIGH);
    break;
  case PINMODE_ANALOG:
    pinMode(pin, INPUT);
    digitalWrite(pin, LOW);
    analogRead(pin);
    break;
  case PINMODE_5V:
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    break;
  case PINMODE_GND:
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    break;
  }
}