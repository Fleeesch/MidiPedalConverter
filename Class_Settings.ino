// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Class : Settings
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

#include "Class_Settings.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Initialization
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// 14 bit
bool Settings::MIDI_USE_14BIT = false;

// interpolation
float Settings::MIDI_INTERPOLATION = 0.5;
int Settings::M_INT_LAST = -1;

bool Settings::M_INT_FREEZE = false;
int Settings::M_INT_FREEZE_PNT = 0;

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Init
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void Settings::init()
{

  // 14 bit setting pin
  pinMode(PIN_SETTING_14BIT, INPUT_PULLUP);
  digitalWrite(PIN_SETTING_14BIT, HIGH);

  pinMode(PIN_SETTING_INTERPOLATION, INPUT);
  digitalWrite(PIN_SETTING_INTERPOLATION, LOW);
  analogRead(PIN_SETTING_INTERPOLATION);

  // update settings once
  Settings::update();
}

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  Method : Update
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void Settings::update()
{

  // :::::::::::::::::::::::::::::::::::
  //    14 Bit

  // store 14 bit setting
  Settings::MIDI_USE_14BIT = !digitalRead(PIN_SETTING_14BIT);

  // :::::::::::::::::::::::::::::::::::
  //    Interpolation Setting
  
  // interpolation
  int interpolation_read = analogRead(PIN_SETTING_INTERPOLATION);
  
  // check for stable signal
  if (interpolation_read == Settings::M_INT_LAST)
  {
    Settings::M_INT_FREEZE = true; // freeze interpolation
    Settings::M_INT_FREEZE_PNT = interpolation_read; // store value point
  }
  
  // reset freeze if threshold is breached
  if (Settings::M_INT_FREEZE && abs(interpolation_read - Settings::M_INT_FREEZE_PNT) > M_INT_CHANGE_THRESH)
  {
    M_INT_FREEZE = false;
  }
  
  // calculate interpolation value
  if (!Settings::M_INT_FREEZE)
  {
    Settings::MIDI_INTERPOLATION = pow(analogRead(PIN_SETTING_INTERPOLATION) / 1024.0, 4) * (1.0 - INTERPOLATION_RAISE) + INTERPOLATION_RAISE;
  }
  
  // store last interpolation point
  Settings::M_INT_LAST = interpolation_read;
  
}
