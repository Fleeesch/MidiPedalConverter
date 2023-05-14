#line 1 "C:\\root\\int\\developement\\arduino\\MidiPedalConverter\\MidiPedalConverter.h"
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Header File
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef MidiPedalConverter_h
#define MidiPedalConverter_h

#include <Arduino.h>
#include <MIDI.h>
#include "Class_MidiHandler.h"
#include "Class_PedalInterface.h"
#include "Class_Pedal.h"
#include "Class_PedalSustain.h"
#include "Class_PedalExpression.h"
#include "Class_AudioJack.h"
#include "Class_VoltageGenerator.h"
#include "Class_MidiPort.h"
#include "Class_MidiPortSerial.h"
#include "Class_MidiPortUsb.h"



// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Pin Configuration
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// voltage generator pin
#define PIN_V_GENERATOR_TIP 38
#define PIN_V_GENERATOR_TIP_DETECT 39
#define PIN_V_GENERATOR_SLEEVE_DETECT 40

// interface 1
#define PIN_INTERFACE_0_TIP A0
#define PIN_INTERFACE_0_TIP_DETECT 22
#define PIN_INTERFACE_0_RING 23
#define PIN_INTERFACE_0_RING_DETECT 24 
#define PIN_INTERFACE_0_SLEEVE_DETECT 25

// interface 2
#define PIN_INTERFACE_1_TIP A1
#define PIN_INTERFACE_1_TIP_DETECT 30
#define PIN_INTERFACE_1_RING 31
#define PIN_INTERFACE_1_RING_DETECT 32
#define PIN_INTERFACE_1_SLEEVE_DETECT 33

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Settings
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#define TEST_MODE false;

#define INTERFACE_COUNT 2;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Setup MIDI
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


MIDI_CREATE_DEFAULT_INSTANCE();


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Variable Declaration
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// voltage generator
VoltageGenerator *VOLTAGE_GENERATOR;

// midi port jack din5-jack
MidiPort *midi_port_jack;
MidiPort *midi_port_usb;

// pedal interfaces
const int INTERFACES_COUNT = INTERFACE_COUNT;
PedalInterface *p_interfaces[INTERFACES_COUNT];

// test mode flag
const bool test_mode = TEST_MODE;

#endif