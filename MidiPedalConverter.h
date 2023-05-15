// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Header File
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef MidiPedalConverter_h
#define MidiPedalConverter_h

#include <Arduino.h>
#include <MIDI.h>

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Pin Configuration
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// settings
#define PIN_SETTING_14BIT 8
#define PIN_SETTING_INTERPOLATION A8

// interface 1
#define PIN_INTERFACE_0_TIP A0
#define PIN_INTERFACE_0_SLEEVE 22
#define PIN_INTERFACE_0_SLEEVE_DETECT 23
#define PIN_INTERFACE_0_RING 24
#define PIN_INTERFACE_0_RING_DETECT 25
#define PIN_INTERFACE_0_TIP_DETECT 27

// interface 2
#define PIN_INTERFACE_1_TIP A1
#define PIN_INTERFACE_1_SLEEVE 32
#define PIN_INTERFACE_1_SLEEVE_DETECT 33
#define PIN_INTERFACE_1_RING 34
#define PIN_INTERFACE_1_RING_DETECT 35
#define PIN_INTERFACE_1_TIP_DETECT 37

// interface 3
#define PIN_INTERFACE_2_TIP A2
#define PIN_INTERFACE_2_SLEEVE 42
#define PIN_INTERFACE_2_SLEEVE_DETECT 43
#define PIN_INTERFACE_2_RING 44
#define PIN_INTERFACE_2_RING_DETECT 45
#define PIN_INTERFACE_2_TIP_DETECT 47

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Settings
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// use test mode (sends serial sensor data)
#define SETTING_TEST_MODE false

// count of interfaces
#define INTERFACE_COUNT 3

// startup delay before anything happens
#define SETTING_STARTUP_DELAY 1000

// pedal detection delay after each successful plugin
#define SETTING_PEDAL_DETECTION_DELAY 1000

// midi start delay after pedal initialization
#define SETTING_PEDAL_MIDI_DELAY 1000

// minimum microseconds between loop iterations
const int LOOP_TIME_MIN = 100;

// only call settings this amount iterations
const int SETTING_CALL_ITERATIONS = 10;

const float INTERPOLATION_RAISE = 0.01;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  INcludes
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "Class_Settings.h"
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
//  Setup MIDI
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// currently the only midi over usb instance
MIDI_CREATE_DEFAULT_INSTANCE();

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Variable Declaration
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// midi port jack din5-jack
MidiPort *midi_port_jack;
MidiPort *midi_port_usb;

// pedal interfaces
const int INTERFACES_COUNT = INTERFACE_COUNT;
PedalInterface *p_interfaces[INTERFACES_COUNT];

// test mode flag
const bool test_mode = SETTING_TEST_MODE;

// for calculating time delta in main loop
long MICROS_LAST = 0;

long LOOP_COUNTER = 0;

#endif