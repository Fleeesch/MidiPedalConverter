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
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Routine
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::routine()
{

  // store sensor input
  int val_target = pedal_interface->audio_jack->getExpressionSignal();

  _value_min = min(val_target, _value_min);
  _value_max = max(val_target, _value_max);

  val_target = map(val_target, _value_min, _value_max, 0, 1023);

  // check if target value is within timeout threshold
  bool is_in_threshold = abs(val_target - _timeout_val) <= _timeout_threshold;

  // timeout is active
  if (_timeout)
  {

    // signal is within threshold
    if (is_in_threshold)
    {
      return; // return signal
    }

    // signal outside of threshold?
    if (!is_in_threshold)
    {

      // reset counter
      _timeout_counter = _timeout_ticks;

      // deactivate timeout
      _timeout = false;
    }
  }

  // decrement timeout
  if (_timeout_counter > 0)
    _timeout_counter--;

  // activate timeout when counter reaches 0
  if (_timeout_counter <= 0)
    _timeout = true;

  // target value differs from last sent MIDI?
  if (_value_last != val_target)
  {

    // reset timeout counter
    _timeout_counter = _timeout_ticks;

    // store timeout value
    _timeout_val = val_target;

    // interpolate from last value
    _value_last += (val_target - _value_last) * _interpolation_factor;

    // try sending MIDI of last value<
    sendMidi(round(_value_last));
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Midi
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::sendMidi(int value)
{

  // send either 7 or 14 bit
  if (!is_14bit)
  {

    // 7 bit message, downsample source value
    sendMidi7Bit(value >> 3);
  }
  else
  {

    // 14 bit, upsample source value
    int val_hires = value << 4;

    // filter MSB and LSB, send MIDI
    sendMidi14Bit(val_hires & 127, val_hires >> 7);
  }
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Midi (7 Bit)
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::sendMidi7Bit(int value)
{

  // skip if value doesn't differ from the last one

  // send midi
  if (last_midi_value != value)
    Midi::sendToAllPorts(getStatusMessage(), getControlChange(), value);

  // store sent midi value
  last_midi_value = value;
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Send Midi (14 Bit)
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::sendMidi14Bit(int lsb, int msb)
{

  // send lsb
  if (last_midi_value_lsb != lsb || last_midi_value != msb)
  {
    Midi::sendToAllPorts(getStatusMessage(), getControlChangeLSB(), lsb);
  }

  // send msb
  if (last_midi_value != msb)
  {
    Midi::sendToAllPorts(getStatusMessage(), getControlChange(), msb);
  }

  last_midi_value = msb;
  last_midi_value_lsb = lsb;
}