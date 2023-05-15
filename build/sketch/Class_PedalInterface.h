#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\Class_PedalInterface.h"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Pedal Interface : Header
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef Class_PedalInterface_h
#define Class_PedalInterface_h

#include <Arduino.h>
#include "MidiPedalConverter.h"
#include "Class_VoltageGenerator.h"
#include "Class_AudioJack.h"
#include "Class_Pedal.h"
#include "Class_PedalSustain.h"
#include "Class_PedalExpression.h"

// interface mode index numbers
#define MODE_DETECTION 1
#define MODE_SUSTAIN 2
#define MODE_EXPRESSION 3
#define MODE_CONTROL_VOLTAGE 4
#define MODE_VOLTAGE_SOURCE 5

// midi info messages lookup
#define MESSAGE_DETECTING 0
#define MESSAGE_ACITVATED 1
#define MESSAGE_MIDI_GO 2

// debounce time for connector detection in ticks
const unsigned int DETECTION_DEBOUNCE_TICKS = SETTING_PEDAL_DETECTION_DELAY;

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Constructor
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

class PedalInterface
{

public:
  PedalInterface();

  // set MIDI channels automatically
  static void automaticMidiChannel();

  // lookup array and index
  static PedalInterface *lookup[];
  static int lookup_index;

  // single audio jack
  AudioJack *audio_jack = NULL;

  // connected pedal
  Pedal *pedal = NULL;
  PedalSustain pedal_sustain = PedalSustain(this);
  PedalExpression pedal_expression = PedalExpression(this);

  // called by main loop
  void routine();

  // test (print data)
  void printPedalData(int index = 0);

  // gets triggered on TRS tip switch change
  bool detectSleeveConnection();

  // reset interrupt debounce counter
  void resetDetectionDebounce();

  // set mode (pin and behaviour configuration)
  void setModeDetection();
  void setModeSustain();
  void setModeExpression();
  void setModeControlVoltage();
  void setModeVoltageSource();

  // mode getter
  bool isSustain();
  bool isExpression();

  // system exclusive status messages
  void sendMidiInfoMessage(int message);

  // set mode by its index number
  void setModeByIndex(int idx);

  // detects the type of pedal that is connected
  void analyzePedalType();

  // returns common interface MIDI channel
  int getCommonMidiChannel();

  // sets the common MIDI channel
  void setCommonMidiChannel(int channel);

  // adds a TRS audio jack with detection switches
  void addAudioJack(int pdt, int pdr, int pds, int pt, int pr, int ps);

  // go back to previous mode
  void revertMode();

  // apply settings of current mode
  void applyCurrentMode();

  // pedal type for testing purpoes
  int test_pedaltype = 0;

  int index = 0;

private:
  // store (and set) mode according to index
  int _storeMode(int mode_set);

  // uses the tip connection switch to detect a plugged-in audio connector
  void _checkConnection();

  // interface common MIDI channel
  int _midi_channel = 0;

  // modes; 1 = detection, 2 = sustain, 2 = expression
  int _mode = NULL;

  // last mode to detect actual mode changes
  int _mode_last = NULL;

  // debounce counter
  int _detection_debounce = 0;

  // presence of audio connector
  bool _insert_state = false;
};

#endif