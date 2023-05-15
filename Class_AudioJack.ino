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
  setPinMode(_pin_detect_ring, PINMODE_DIGITAL);
  setPinMode(_pin_detect_sleeve, PINMODE_PULLUP);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Get Pluck Signal
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool AudioJack::getPluckSignal()
{

  return digitalRead(_pin_detect_sleeve);
}

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

  // pointers to interface and its jack
  PedalInterface *p_int;
  AudioJack *jack;

  // go through pedals
  for (int i = 0; i < PedalInterface::lookup_index; i++)
  {

    // get jack of interace
    p_int = PedalInterface::lookup[i];
    jack = p_int->audio_jack;

    // set jack to GND
    jack->setupGND();
  }

  for (int i = 0; i < PedalInterface::lookup_index; i++)
  {
    // get pedal interface
    p_int = PedalInterface::lookup[i];

    // get jack
    jack = p_int->audio_jack;

    // skip itself, or if there's no audio jack
    if (!jack || jack == this)
      continue;

    // startup values for analog read signal
    int a_read = 1023;
    int a_read_last = a_read;

    // fail indicator
    bool fail = false;

    // do the handshake test
    for (int i = 0; i < 100; i++)
    {

      // get even / uneven iteration index state
      int mod_val = i % 2;
      
      // set to 5V / GND depending on index even number
      if (!mod_val)
        jack->setupGND();
      else
        jack->setup5V();
      
      // stabilization delay
      delayMicroseconds(5);
      
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

      // fail, skip test
      fail = true;
      break;
    }
    
    // test was a success?
    if (!fail)
    {
      // set mode to voltage source
      p_int->setModeVoltageSource();
      return true;
    }
    
    // check for interface failed, load last jack configuration
    p_int->applyCurrentMode();
  }
  
  // all tests failed, no 5V connection
  return false;
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Get Mono Test Signal
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool AudioJack::getSustainTestSignal()
{

  return digitalRead(_pin_sleeve);
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
  setPinMode(_pin_sleeve, PINMODE_DIGITAL);
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
//  Method : Setup 5V
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setup5V()
{

  setPinMode(_pin_tip, PINMODE_5V);
  setPinMode(_pin_ring, PINMODE_GND);
  setPinMode(_pin_sleeve, PINMODE_GND);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Setup GND
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void AudioJack::setupGND()
{

  setPinMode(_pin_tip, PINMODE_GND);
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