#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "Hardware.h"
#include "Midi.h"
#include "HD44780.h"
#include "Globals.h"

// Functions declarations
void scanKeyboard(void);
void toggleMode(void);
bool toggleModeRequested(void);
bool toggleModeAllowed(void);

#endif // _KEYBOARD_H_