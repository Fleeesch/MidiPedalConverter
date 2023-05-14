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
  _value_min = 110;
  _value_max = 1000;
  _value_last = 0;
  _target_last = 0;
  _timeout_counter = 0;
  _last_midi_value_msb = 0;
  _last_midi_value_lsb = 0;

  init_millis = millis();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Routine
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void PedalExpression::routine()
{
  // midi initialisation
  if (!midiIsGo())
    ;

  // store sensor input
  int _value_target = pedal_interface->audio_jack->getExpressionSignal();

  // recalculate boundaries
  _value_min = min(_value_target + 1, _value_min);
  _value_max = max(_value_target - 1, _value_max);

  // rescale using minimal and maximal values
  _value_target = map(_value_target, _value_min, _value_max, 0, 1023);

  // check if delta is below threshold value
  bool is_in_threshold = abs(_value_target - _target_last) <= _timeout_threshold;

  _target_last = _value_target;

  // timeout active?
  if (_timeout)
  {
    // reset timeout if within treshold
    if (!is_in_threshold)
    {

      _timeout = false;
      _timeout_counter = 0;
    }

    // skip the rest of the code
    if (is_in_threshold)
      return;
  }

  // check if signal is within threshold
  if (is_in_threshold)
  {

    // increment timeout counter
    if (_timeout_counter <= _timeout_ticks)
      _timeout_counter++;

    // timeout activation
    if (_timeout_counter == _timeout_ticks)
      _timeout = true;
  }
  else
  {
    // reset timeout counter when threshold breached
    _timeout_counter = 0;
  }

  // interpolate from last value
  _value_last += (_value_target - _value_last) * _interpolation_factor;

  // send midi
  sendMidi(round(_value_last));
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
  if (!is_14bit)
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
  {
    MidiHandler::sendToAllPorts(getStatusMessage(), getControlChangeLSB(), lsb);
  }

  // send msb
  if (_last_midi_value_msb != msb)
  {
    MidiHandler::sendToAllPorts(getStatusMessage(), getControlChange(), msb);
  }

  _last_midi_value_msb = msb;
  _last_midi_value_lsb = lsb;
}