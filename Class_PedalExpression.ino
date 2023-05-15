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