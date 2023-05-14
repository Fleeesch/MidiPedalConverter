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
  pedal->routine();
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

void PedalInterface::_revertMode()
{

  setModeByIndex(_mode_last);
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
  
  // [!] this one's a hack since pedal-type detection is not working reliably
  
  switch (index){
    case 0:
      setModeSustain();
    break;
    case 1:
      setModeExpression();
    break;
  
  }
  
  return; // ---> Skip the Rest
  
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
    VOLTAGE_GENERATOR->setState(1);
    audio_jack->setupControlVoltage();
    Serial.print("CV, ");
    Serial.print(audio_jack->getControlVoltageSignal());
    break;
  }

  Serial.println();
}