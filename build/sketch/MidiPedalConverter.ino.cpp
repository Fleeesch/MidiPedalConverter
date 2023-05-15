#include <Arduino.h>
#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\MidiPedalConverter.ino"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//  MIDI PEDAL CONVERTER
//
//  v1.0
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Dependencies
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "MidiPedalConverter.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Setup
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#line 21 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\MidiPedalConverter.ino"
void setup();
#line 154 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\MidiPedalConverter.ino"
void loop();
#line 21 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\MidiPedalConverter.ino"
void setup()
{

  // initialize settings
  Settings::init();

  // start midi communication
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // setup pedal interfaces
  for (int i = 0; i < INTERFACES_COUNT; i++)
  {
    p_interfaces[i] = new PedalInterface();

    switch (i)
    {

    case 0:
      p_interfaces[i]->addAudioJack(
          PIN_INTERFACE_0_TIP_DETECT,
          PIN_INTERFACE_0_RING_DETECT,
          PIN_INTERFACE_0_SLEEVE_DETECT,
          PIN_INTERFACE_0_TIP,
          PIN_INTERFACE_0_RING,
          PIN_INTERFACE_0_SLEEVE);
      break;

    case 1:
      p_interfaces[i]->addAudioJack(
          PIN_INTERFACE_1_TIP_DETECT,
          PIN_INTERFACE_1_RING_DETECT,
          PIN_INTERFACE_1_SLEEVE_DETECT,
          PIN_INTERFACE_1_TIP,
          PIN_INTERFACE_1_RING,
          PIN_INTERFACE_1_SLEEVE);
      break;

    case 2:
      p_interfaces[i]->addAudioJack(
          PIN_INTERFACE_2_TIP_DETECT,
          PIN_INTERFACE_2_RING_DETECT,
          PIN_INTERFACE_2_SLEEVE_DETECT,
          PIN_INTERFACE_2_TIP,
          PIN_INTERFACE_2_RING,
          PIN_INTERFACE_2_SLEEVE);
      break;

    default:
      p_interfaces[i]->addAudioJack(0, 0, 0, 0, 0, 0);
      break;
    }

    p_interfaces[i]->setModeDetection();
  }

  // ::::::::::::::::::
  //    Test Mode
  // ::::::::::::::::::

  if (test_mode)
  {

    // Start Serial Connection
    Serial.begin(9600);

    // print introduction lines
    Serial.println("::: TEST MODE :::");
    delay(500);
    Serial.println("Analyzing Pedal Type...");

    // go through interfaces
    for (int i = 0; i < INTERFACES_COUNT; i++)
    {

      // test for control voltage
      p_interfaces[i]->audio_jack->setupControlVoltage();

      // control voltage found? Use pedal type 2, skip the rest
      if (p_interfaces[i]->audio_jack->testForControlVoltage())
      {
        p_interfaces[i]->test_pedaltype = 2;
        continue;
      }

      // test for Sustain Signal
      p_interfaces[i]->audio_jack->setupSustainTest();

      // get test signal
      if (p_interfaces[i]->audio_jack->getSustainTestSignal())
      {
        // pedal is a sustain type
        p_interfaces[i]->test_pedaltype = 0;
        continue;
      }
      else
      {
        // pedal is an expression type
        p_interfaces[i]->test_pedaltype = 1;

        continue;
      }
    }

    // analyzation done
    delay(500);
    Serial.println("Analyze Complete! ...");
  }

  // ::::::::::::::::::
  //    MIDI Mode
  // ::::::::::::::::::

  if (!test_mode)
  {

    // create MIDI Din5 port, add to lookup list
    midi_port_jack = new MidiPortSerial(Serial);
    MidiHandler::addPort(midi_port_jack);

    // create MIDI USB port, add to lookup list
    midi_port_usb = new MidiPortUsb();
    MidiHandler::addPort(midi_port_usb);

    Serial.begin(31250);
  }

  delay(SETTING_STARTUP_DELAY);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Loop
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void loop()
{

  int time_delta = micros() - MICROS_LAST;

  if (LOOP_TIME_MIN && time_delta < LOOP_TIME_MIN)
  {
    delayMicroseconds(LOOP_TIME_MIN - time_delta);
  }

  // ::::::::::::::::::
  //    Test Mode
  // ::::::::::::::::::

  if (test_mode)
  {

    // go through pedal interfaces, test
    for (int i = 0; i < INTERFACES_COUNT; i++)
    {
      p_interfaces[i]->printPedalData(i);
    }
    Serial.println("---");
    delay(2000);

    return;
  }

  // ::::::::::::::::::
  //    MIDI Mode
  // ::::::::::::::::::

  // go through pedal interfaces, do routines
  for (int i = 0; i < INTERFACES_COUNT; i++)
  {
    p_interfaces[i]->routine();
  }

  // ::::::::::::::::::
  //    Settings
  // ::::::::::::::::::

  // only call settings after certain amount of iterations
  if (!(LOOP_COUNTER % SETTING_CALL_ITERATIONS))
    Settings::update();

  // increment / reset loop counter
  if (LOOP_COUNTER++ >= 10000000)
    LOOP_COUNTER = 0;

  // store time
  MICROS_LAST = micros();
}

#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_AudioJack.ino"
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
#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_MidiHandler.ino"
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
#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_MidiPort.ino"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : MIDI Port
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include "Class_MidiPort.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

MidiPort::MidiPort(){

};

#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_MidiPortSerial.ino"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : MIDI Port : Serial
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include "Class_MidiPortSerial.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

MidiPortSerial::MidiPortSerial(Stream &serialport)
{

  // store serial port reference
  _serialport = serialport;
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Message
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void MidiPortSerial::sendMessage(int b1, int b2, int b3)
{

  // mandatory first two bytes
  _serialport.write(b1);
  _serialport.write(b2);

  // last byte is optional
  if (b3 >= 0)
    _serialport.write(b3);
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Byte
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void MidiPortSerial::sendByte(int b)
{
  
  _serialport.write(b);
}
#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_MidiPortUsb.ino"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : MIDI Port : USB
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include <Midi.h>
#include "Class_MidiPortUsb.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

MidiPortUsb::MidiPortUsb(){
  
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Message
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void MidiPortUsb::sendMessage(int b1, int b2, int b3)
{
  
  // [!] ignore 2 byte messages for now
  if (b3 >= 0)
    return;
        
        // send midi
       
    MIDI.sendControlChange(b2, b3, b1 & 0x0F);
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Byte
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void MidiPortUsb::sendByte(int b)
{
}
#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_Pedal.ino"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Pedal
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include "Class_Pedal.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

Pedal::Pedal()
{
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Reset
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void Pedal::reset()
{

  // system exclusive info message
  pedal_interface->sendMidiInfoMessage(MESSAGE_ACITVATED);

  // reset midi timer
  init_millis = millis();
  midi_is_go = false;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

Pedal::Pedal(PedalInterface *p_interface)
{

  pedal_interface = p_interface;
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Routine
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void Pedal::routine()
{
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Get Status Message
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

int Pedal::getStatusMessage()
{

  return 0xB0 + pedal_interface->getCommonMidiChannel();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : MIDI is Go
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool Pedal::midiIsGo()
{
  
  if (millis() - init_millis < midi_message_dismiss_time)
  {
    midi_is_go = false;
    return false;
  }
  else
  {

    if (!midi_is_go)
    {

      pedal_interface->sendMidiInfoMessage(MESSAGE_MIDI_GO);

      midi_is_go = true;
    }

    return true;
  }
}
#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_PedalExpression.ino"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Pedal : Expression
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include "Class_PedalExpression.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

PedalExpression::PedalExpression(PedalInterface *pedal_if)
    : Pedal(pedal_if)
{

  reset();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Reset
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::reset()
{

  Pedal::reset();

  // reset values
  _value_min = EXPRESSION_PEDAL_LOW;
  _value_max = EXPRESSION_PEDAL_HIGH;

  _value_last = 0;
  _target_last = 0;

  _last_midi_value_msb = 0;
  _last_midi_value_lsb = 0;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Routine
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::routine()
{
  // midi initialisation
  if (!midiIsGo())
    return;

  // store last target value
  _target_last = _value_target;
  
  // store sensor input
  _value_target = pedal_interface->audio_jack->getExpressionSignal();
  
  // recalculate boundaries
  _value_min = min(_value_target + 1, _value_min);
  _value_max = max(_value_target - 1, _value_max);
  
  // rescale using minimal and maximal values
  _value_target = map(_value_target, _value_min, _value_max, 0, 1023);
  
  // handle freeze, keep old value if it still applies
  if (!_escapeFreeze() || _applyFreeze())
    _value_target = _target_last;
  
  // interpolate from last value
  _value_last += (_value_target - _value_last) * Settings::MIDI_INTERPOLATION;
  
  // send midi
  sendMidi(round(_value_last));
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Escape Freeze
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool PedalExpression::_escapeFreeze()
{

  // don't do anything if threre's no freeze
  if (!_freeze)
    return true;

  // check if freeze threshold has been breached
  if (abs(_value_target - _freeze_value) > _FREEZE_THRESHOLD)
  {

    // turn of freeze
    _freeze = false;

    // max out delta to prevent re-freezing
    _value_delta = _VALUE_DELTA_CAP;

    // zero out  following delta increment
    _target_last = _value_target;

    // freeze escaped!
    return true;
  }
  else
  {
    // still stuck in freeze
    return false;
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Calculate Delta
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool PedalExpression::_applyFreeze()
{
  
  // calculate delta, boost via factor to keep it floating for a short time
  _value_delta += (_value_target - _target_last) * 2;

  // increment delta if negative
  if (_value_delta < 0)
    _value_delta++;

  // decrement delta if positive
  if (_value_delta > 0)
    _value_delta--;

  // keep delta within limit (no excessive delta floating)
  _value_delta = min(max(_value_delta, -_VALUE_DELTA_CAP), _VALUE_DELTA_CAP);

  // delta is to low, indicating no movement?
  if (abs(_value_delta) < _VALUE_DELTA_THRESHOLD)
  {
    // freeze
    _freeze = true;

    // store point where freeze happened
    _freeze_value = _value_target;

    // freeze happened!
    return true;
  }

  // no freeze happened
  return false;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Midi
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::sendMidi(int value)
{

  // skip if not enough time has passed since initialization
  if (!midi_is_go)
    return;

  // send either 7 or 14 bit
  if (!Settings::MIDI_USE_14BIT) // if (!Settings::MIDI_USE_14BIT)
  {
    sendMidi7Bit(value >> 3); // 7 bit message, downsample source value
  }
  else
  {
    int val_hires = value << 4;                     // 14 bit, upsample source value
    sendMidi14Bit(val_hires & 127, val_hires >> 7); // filter MSB and LSB, send MIDI
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Midi (7 Bit)
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::sendMidi7Bit(int msb)
{

  // limit values
  msb = min(max(msb, 0), 127);

  // send midi if value is different
  if (_last_midi_value_msb != msb)
    MidiHandler::sendToAllPorts(getStatusMessage(), getControlChange(), msb);

  // store sent midi value
  _last_midi_value_msb = msb;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Midi (14 Bit)
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::sendMidi14Bit(int lsb, int msb)
{

  // limit values
  msb = min(max(msb, 0), 127);
  lsb = min(max(lsb, 0), 127);

  // send lsb
  if (_last_midi_value_lsb != lsb || _last_midi_value_msb != msb)
    MidiHandler::sendToAllPorts(getStatusMessage(), getControlChangeLSB(), lsb);

  // send msb
  if (_last_midi_value_msb != msb)
    MidiHandler::sendToAllPorts(getStatusMessage(), getControlChange(), msb);

  _last_midi_value_msb = msb;
  _last_midi_value_lsb = lsb;
}
#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_PedalInterface.ino"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Pedal Interface
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include "Class_PedalInterface.h"

// lookup array and index
PedalInterface *PedalInterface::lookup[16];
int PedalInterface::lookup_index = 0;

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

PedalInterface::PedalInterface()
{

  // skip if maximal interface count has been reached
  if (lookup_index >= 16)
    return;

  // add interace to lookup
  PedalInterface::lookup[PedalInterface::lookup_index] = this;

  // store index number
  index = lookup_index;

  // increment index
  PedalInterface::lookup_index++;
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Detect Sleeve Connection
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool PedalInterface::detectSleeveConnection()
{

  return audio_jack->getPluckSignal();
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Static Method : Automatic MIDI Channel
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::automaticMidiChannel()
{

  // pedal type channel incrementors
  int chan_exp = 0;
  int chan_sus = 0;

  // pedal reference
  PedalInterface *p_int;

  // go through pedal interfaces
  for (int i = 0; i < PedalInterface::lookup_index; i++)
  {
    // get pedal interface
    p_int = PedalInterface::lookup[i];

    // interface is set to sustain?
    if (p_int->isSustain())
    {
      p_int->setCommonMidiChannel(chan_sus % 16); // set midi channel (1 - 16)
      chan_sus++;                                 // increment channel
    }

    // interface is set to expression?
    if (p_int->isExpression())
    {
      p_int->setCommonMidiChannel(chan_exp % 16); // set midi channel (1 - 16)
      chan_exp++;                                 // increment channel
    }
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Routine
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::routine()
{

  // check if the pluck is missing
  if (!detectSleeveConnection())
  {

    // reset plugin debouncing
    resetDetectionDebounce();

    // set mode to detection
    setModeDetection();
  }

  // skip routine if detection mode is active
  if (_mode == MODE_DETECTION)
  {

    // update audio connector data
    _checkConnection();

    // audio connector is plugged in?
    if (_insert_state)
    {

      // increment debounce
      if (_detection_debounce <= DETECTION_DEBOUNCE_TICKS)
        _detection_debounce++;

      // analyze pedal type one-time when debounce has been successful
      if (_detection_debounce == DETECTION_DEBOUNCE_TICKS)
        analyzePedalType();

      // no audio connector plugged in?
    }

    if (!_insert_state)
      resetDetectionDebounce(); // reset debounce

    return; // skip normal routine
  }

  // pedal routine only
  if (pedal)
  {
    pedal->routine();
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Add Audio Jack
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::addAudioJack(int pdt, int pdr, int pds, int pt, int pr, int ps)
{

  audio_jack = new AudioJack(pdt, pdr, pds, pt, pr, ps);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Mode to Detection
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::setModeDetection()
{

  // skip if mode stays the same
  if (_mode == MODE_DETECTION)
    return;

  // store mode
  _storeMode(MODE_DETECTION);

  // reset pedal if there is one
  if (pedal)
  {
    pedal->reset();
  }

  // configure IO
  audio_jack->setupDetection();

  // reset debounce counter
  resetDetectionDebounce();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Mode to Sustain
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::setModeSustain()
{

  // skip if mode stays the same
  if (_mode == MODE_SUSTAIN)
    return;

  // store mode
  _storeMode(MODE_SUSTAIN);

  // configure IO
  audio_jack->setupSustain();

  // redirect pedal
  pedal = &pedal_sustain;
  pedal->reset();

  // apply automtic MIDI channels
  PedalInterface::automaticMidiChannel();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Mode to Expression
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::setModeExpression()
{

  // skip if mode stays the same
  if (_mode == MODE_EXPRESSION)
    return;

  // store mode
  _storeMode(MODE_EXPRESSION);

  // configure IO
  audio_jack->setupExpression();

  // redirect pedal
  pedal = &pedal_expression;
  pedal->reset();

  // apply automtic MIDI channels
  PedalInterface::automaticMidiChannel();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Mode to Control Voltage
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::setModeControlVoltage()
{

  if (_mode == MODE_CONTROL_VOLTAGE)
    return;

  _storeMode(MODE_CONTROL_VOLTAGE);
  audio_jack->setupControlVoltage();

  // redirect pedal
  pedal = &pedal_expression;
  pedal->reset();

  // apply automtic MIDI channels
  PedalInterface::automaticMidiChannel();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Mode to Control Voltage
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::setModeVoltageSource()
{

  if (_mode == MODE_VOLTAGE_SOURCE)
    return;

  _storeMode(MODE_VOLTAGE_SOURCE);
  audio_jack->setup5V();

  // reset pedal
  pedal = NULL;

  // apply automtic MIDI channels
  PedalInterface::automaticMidiChannel();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Mode by Index
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::setModeByIndex(int idx)
{

  // pick mode by index
  switch (idx)
  {
  case MODE_DETECTION:
    setModeDetection();
    break;
  case MODE_SUSTAIN:
    setModeSustain();
    break;
  case MODE_EXPRESSION:
    setModeExpression();
    break;
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Store Mode
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

int PedalInterface::_storeMode(int mode_set)
{

  // store mode
  _mode = mode_set;

  // store last mode if new mode is different
  if (_mode_last != _mode)
    _mode_last = _mode;

  // return the new mode number
  return _mode;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Revert Mode
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::revertMode()
{
  // use stored mode index
  setModeByIndex(_mode_last);
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Apply Current Mode
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::applyCurrentMode()
{
  // save current mode
  int m = _mode;

  // reset mode number, forcing a mode change
  _mode = 0;

  // apply mode
  setModeByIndex(m);
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Check Connection
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::_checkConnection()
{

  // audio jac missing? skip
  if (!audio_jack)
    return;

  // tip is connected?
  if (audio_jack->getDectectionSignal())
  {

    _insert_state = true;
  }
  else
  {

    _insert_state = false;
  }
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Analyze Pedal Type
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::analyzePedalType()
{

  audio_jack->setupControlVoltage();

  // check for volume pedal
  if (audio_jack->testForControlVoltage())
  {

    // Pedal is Volume Pedal
    setModeControlVoltage();
    return; // -> Skip Rest
  }
  else
  {

    // prepare for mono connector test
    audio_jack->setupSustainTest();

    // check for mono connector
    if (audio_jack->getSustainTestSignal())
    {

      // Pedal is Sustain Pedal
      setModeSustain();
      return; // -> Skip Rest
    }
    else
    {

      // Pedal is Expression Pedal
      setModeExpression();
      return; // -> Skip Rest
    }
  }

  // Nothing found, back to Detection Mode
  audio_jack->setupDetection();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Reset Interrupt Debounce
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::resetDetectionDebounce()
{

  _detection_debounce = 0;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Get Common MIDI Channel
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

int PedalInterface::getCommonMidiChannel()
{
  return _midi_channel;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Common MIDI Channel
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::setCommonMidiChannel(int channel)
{

  _midi_channel = min(max(channel, 0), 15);
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Is Sustain
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool PedalInterface::isSustain()
{
  return _mode == MODE_SUSTAIN;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Is Expression
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

bool PedalInterface::isExpression()
{
  return _mode == MODE_EXPRESSION || _mode == MODE_CONTROL_VOLTAGE;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Midi Info Message
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::sendMidiInfoMessage(int message)
{

  int message_out[100]; // pre-declare message
  int msg_l = 0;        // length gets set manually

  switch (message)
  {

  // activation message
  case MESSAGE_ACITVATED:

    message_out[0] = 0x01;
    message_out[1] = index;
    message_out[2] = _mode;
    msg_l = 3;

    break;

  // detection message
  case MESSAGE_DETECTING:

    message_out[0] = 0xF0;
    message_out[1] = index;
    msg_l = 2;

    break;

  // midi activation message
  case MESSAGE_MIDI_GO:
    message_out[0] = 0x0B;
    message_out[1] = index;
    msg_l = 2;

    break;
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Test
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalInterface::printPedalData(int index)
{

  // print pedal number
  Serial.print("Pedal [");
  Serial.print(index);
  Serial.print("]: ");

  // test mode for insertion
  audio_jack->setupDetection();

  // check if connector is inserted
  if (!audio_jack->getDectectionSignal())
  {

    // print palceholder, skip rest
    Serial.print("-");
    Serial.println();
    return;
  }

  // check analyzed pedaltype for test mode
  switch (test_pedaltype)
  {
  // print sustain signal
  case 0:
    audio_jack->setupSustain();
    Serial.print("Sustain, ");
    Serial.print(audio_jack->getSustainSignal());
    break;
  // print expression signal
  case 1:
    audio_jack->setupExpression();
    Serial.print("Expression, ");
    Serial.print(audio_jack->getExpressionSignal());
    break;
  // print control voltage signal
  case 2:
    audio_jack->setupControlVoltage();
    Serial.print("CV, ");
    Serial.print(audio_jack->getControlVoltageSignal());
    break;
  }

  Serial.println();
}
#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_PedalSustain.ino"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Pedal : Sustain
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include <Arduino.h>
#include "Class_PedalSustain.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

PedalSustain::PedalSustain(PedalInterface *pedal_if)
    : Pedal(pedal_if)
{

  reset();
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Reset
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalSustain::reset()
{
  Pedal::reset();

  // reset values
  state = false;
  _signal_debounce = 0;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Routine
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalSustain::routine()
{

  // midi initialisation
  if (!midiIsGo())
    ;

  // :::::::::::::::::::::::
  //   Debouncing
  // :::::::::::::::::::::::

  //   --- Pedal Signal High? ---
  if (pedal_interface->audio_jack->getSustainSignal())
  {

    // increment debounce
    if (_signal_debounce < SUSTAIN_DEBOUNCE_TICKS)
      _signal_debounce++;

    //   --- Pedal Signal Low? ---
  }
  else
  {
    // decrement debounce
    if (_signal_debounce > 0)
      _signal_debounce--;
  }

  // :::::::::::::::::::::::::::
  //   Debounce Result Checkup
  // :::::::::::::::::::::::::::

  // pedal on, debounce successful?
  if (!state && _signal_debounce >= SUSTAIN_DEBOUNCE_TICKS)
  {
    state = true; // update pedal state
    sendMidi(state);
  }

  // pedal off, debounce successful?
  if (state && _signal_debounce <= 0)
  {
    state = false; // update pedal state
    sendMidi(state);
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Midi
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalSustain::sendMidi(bool state)
{

  // skip if not enough time has passed since initialization
  if (!midi_is_go)
    return;

  if (state)
    MidiHandler::sendToAllPorts(getStatusMessage(), getControlChange(), 0x7F); // send MIDI - Sustain On
  else
    MidiHandler::sendToAllPorts(getStatusMessage(), getControlChange(), 0x00); // send MIDI - Sustain Off
}

#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_Settings.ino"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Settings
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include "Class_Settings.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Initialization
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// 14 bit
bool Settings::MIDI_USE_14BIT = false;

// interpolation
float Settings::MIDI_INTERPOLATION = 0.5;
int Settings::M_INT_LAST = -1;

bool Settings::M_INT_FREEZE = false;
int Settings::M_INT_FREEZE_PNT = 0;

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Init
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void Settings::init()
{

  // 14 bit setting pin
  pinMode(PIN_SETTING_14BIT, INPUT_PULLUP);
  digitalWrite(PIN_SETTING_14BIT, HIGH);

  pinMode(PIN_SETTING_INTERPOLATION, INPUT);
  digitalWrite(PIN_SETTING_INTERPOLATION, LOW);
  analogRead(PIN_SETTING_INTERPOLATION);

  // update settings once
  Settings::update();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Update
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void Settings::update()
{

  // :::::::::::::::::::::::::::::::::::
  //    14 Bit

  // store 14 bit setting
  Settings::MIDI_USE_14BIT = !digitalRead(PIN_SETTING_14BIT);

  // :::::::::::::::::::::::::::::::::::
  //    Interpolation Setting
  
  // interpolation
  int interpolation_read = analogRead(PIN_SETTING_INTERPOLATION);
  
  // check for stable signal
  if (interpolation_read == Settings::M_INT_LAST)
  {
    Settings::M_INT_FREEZE = true; // freeze interpolation
    Settings::M_INT_FREEZE_PNT = interpolation_read; // store value point
  }
  
  // reset freeze if threshold is breached
  if (Settings::M_INT_FREEZE && abs(interpolation_read - Settings::M_INT_FREEZE_PNT) > M_INT_CHANGE_THRESH)
  {
    M_INT_FREEZE = false;
  }
  
  // calculate interpolation value
  if (!Settings::M_INT_FREEZE)
  {
    Settings::MIDI_INTERPOLATION = pow(analogRead(PIN_SETTING_INTERPOLATION) / 1024.0, 4) * (1.0 - INTERPOLATION_RAISE) + INTERPOLATION_RAISE;
  }
  
  // store last interpolation point
  Settings::M_INT_LAST = interpolation_read;
  
}

#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_VoltageGenerator.ino"
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

VoltageGenerator::VoltageGenerator()
{

  // set pin to output GND
  pinMode(_pin, OUTPUT);
  analogWrite(_pin, 0);
};

// * * * * * * * * * * * * * * * * * * *
//    With Pin

VoltageGenerator::VoltageGenerator(int pin)
{

  // store given pin number
  _pin = pin;

  // set pin to output GND
  pinMode(_pin, OUTPUT);
  analogWrite(_pin, 0);
};

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set State
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

int VoltageGenerator::getVoltage()
{

  return _voltage;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set State
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void VoltageGenerator::setState(bool state)
{

  if (state)
  {
    analogWrite(_pin, 255);
    _voltage = 5;
  }
  else
  {
    analogWrite(_pin, 0);
    _voltage = 0;
  };
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Set Voltage
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void VoltageGenerator::setVoltage(float volt)
{

  // limit range
  volt = min(max(volt, 0), 5);

  // store internal voltage
  _voltage = volt;

  // expand to 0-255
  volt *= 51;

  // write signal
  analogWrite(_pin, volt);
};
