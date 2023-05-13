// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Header File
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#ifndef MidiPedalConverter_h
#define MidiPedalConverter_h

#include "Class_Midi.h"
#include "Class_PedalInterface.h"
#include "Class_Pedal.h"
#include "Class_PedalSustain.h"
#include "Class_PedalExpression.h"
#include "Class_AudioJack.h"
#include "Class_VoltageGenerator.h"
#include "Class_MidiPort.h"
#include "Class_MidiPortSerial.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Pin Configuration
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// interrupt pins for sleeve connector
#define SLEEVE_INTERRUPT_PEDAL 18
#define SLEEVE_INTERRUPT_PEDAL_ALT 2
#define SLEEVE_INTERRUPT_VOLTAGE 19

// voltage generator pin
#define PIN_V_GENERATOR_TIP 42
#define PIN_V_GENERATOR_TIP_DETECT 44
#define PIN_V_GENERATOR_SLEEVE_DETECT 45

// interface 1
#define PIN_INTERFACE_0_TIP A0
#define PIN_INTERFACE_0_RING 22
#define PIN_INTERFACE_0_TIP_DETECT 25
#define PIN_INTERFACE_0_SLEEVE_DETECT 26

// interface 2
#define PIN_INTERFACE_1_TIP A2
#define PIN_INTERFACE_1_RING 32
#define PIN_INTERFACE_1_TIP_DETECT 35
#define PIN_INTERFACE_1_SLEEVE_DETECT 36

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Settings
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#define TEST_MODE false;

#define INTERFACE_COUNT 2;


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Method Decalaration
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// setup TRS detection interrupt
void setupDetectionInterrupt(int pin);

// method that gets triggered on interrupt
void detectionChange();


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Variable Declaration
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// voltage generator
VoltageGenerator *VOLTAGE_GENERATOR;

// midi port jack din5-jack
MidiPort *midi_port_jack;

// pedal interfaces
const int INTERFACES_COUNT = INTERFACE_COUNT;
PedalInterface *p_interfaces[INTERFACES_COUNT];

// test mode flag
const bool test_mode = TEST_MODE;

#endif