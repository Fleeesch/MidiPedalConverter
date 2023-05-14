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

  _last_direction_millis = millis();
  _last_direction = false;
  
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
  
  

  // : : : : : : : : : : : : : : :
  //  Poti Debouncing
  // : : : : : : : : : : : : : : :
  
  // check for last direction change to the negative
  if (!_last_direction && _value_target > _target_last)
  {
    
    // store direction change data
    _last_direction = true;
    _last_direction_millis = millis();
    
    // direction chagne to fast? skip rest of code
    if (_last_direction_millis < _direction_timeout)
    {
      return;
    }
  }
  
  // check for last direction change to the positive
  if (_last_direction && _value_target < _target_last)
  {
    
    // store direction change data
    _last_direction = false;
    _last_direction_millis = millis();
    
    // direction chagne to fast? skip rest of code
    if (_last_direction_millis < _direction_timeout)
    {
      return;
    }
  }
  
  // store last target value
  _target_last = _value_target;
  
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