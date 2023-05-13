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
    : Pedal(pedal_if){
      
      };

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Routine
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalSustain::routine()
{

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
    Midi::sendToAllPorts(getStatusMessage(), getControlChange(), 0x7F); // send MIDI - Sustain On
    state = true;                                                       // update pedal state
  }

  // pedal off, debounce successful?
  if (state && _signal_debounce <= 0)
  {
    Midi::sendToAllPorts(getStatusMessage(), getControlChange(), 0x00); // send MIDI - Sustain Off
    state = false;                                                      // update pedal state
  }
}
