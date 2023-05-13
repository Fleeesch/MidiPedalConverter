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

#include <Arduino.h>
#include "MidiPedalConverter.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Globals
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Setup
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void setup()
{
  
  
  // setup voltage generator
  VOLTAGE_GENERATOR = new VoltageGenerator(PIN_V_GENERATOR_TIP);
  VOLTAGE_GENERATOR->setState(true);
  
  // setup pedal interfaces
  for (int i = 0; i < INTERFACES_COUNT; i++)
  {
    p_interfaces[i] = new PedalInterface();
    
    switch (i)
    {
    default:
      p_interfaces[i]->addAudioJack(0, 0, 0, 0, 0, 0);
      break;
    
    case 0:
      p_interfaces[i]->addAudioJack(PIN_INTERFACE_0_TIP_DETECT, 0, PIN_INTERFACE_0_SLEEVE_DETECT, PIN_INTERFACE_0_TIP, PIN_INTERFACE_0_RING, 0);
      break;
    
    case 1:
      p_interfaces[i]->addAudioJack(PIN_INTERFACE_1_TIP_DETECT, 0, PIN_INTERFACE_1_SLEEVE_DETECT, PIN_INTERFACE_1_TIP, PIN_INTERFACE_1_RING, 0);
      break;
    }
    
    p_interfaces[i]->setModeDetection();
  }
  
  // detection pin interrupt
  setupDetectionInterrupt(SLEEVE_INTERRUPT_PEDAL);
  setupDetectionInterrupt(SLEEVE_INTERRUPT_PEDAL_ALT);
  setupDetectionInterrupt(SLEEVE_INTERRUPT_VOLTAGE);
  
  
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
    
    // create MIDI port, add to lookup list
    midi_port_jack = new MidiPortSerial(Serial);
    Midi::addPort(midi_port_jack);

    Serial.begin(31250);
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Loop
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void loop()
{
  
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
  
  // go through pedal interfaces, do routines
  for (int i = 0; i < INTERFACES_COUNT; i++)
  {
    p_interfaces[i]->routine();
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Method : Setup Detection Interrrupt
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void setupDetectionInterrupt(int pin)
{
  
  // use pullup
  pinMode(pin, INPUT_PULLUP);
  digitalWrite(pin, HIGH);
  
  // add interrupt
  attachInterrupt(digitalPinToInterrupt(pin), detectionChange, CHANGE);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Interrupt : Detection Change
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void detectionChange()
{
  // show only message in test mode
  if(test_mode){
    Serial.println(">>> Plugin Interrupt");
    return;
  }
  
  // to through interfaces
  for (int i = 0; i < INTERFACES_COUNT; i++)
  {
    // trigger interrupt handler
    p_interfaces[i]->detectionInterrupt();
  }
}