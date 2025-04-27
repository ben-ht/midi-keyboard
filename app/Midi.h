/*
 * MIDI-specific functionality
 */

#ifndef MIDI_H_
#define MIDI_H_

#include "Keyboard.h"
#include "Descriptors.h"

void MIDI_Task(void);
int getMidiNote(int buttonId);
void octaveUp(void);
void octaveDown(void);

#endif // _MIDI_H_