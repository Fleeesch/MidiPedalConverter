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
