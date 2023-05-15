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
