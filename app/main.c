/*
 * Main source file containing the entry point and main loop
 */

#include "Keyboard.h"
#include "Hardware.h"
#include "Midi.h"
#include "Keyboard.h"
#include "HD44780.h"
#include "Globals.h"

/* List of instruments */
Instrument instruments[NB_INSTRUMENTS] = {
    {0, "Piano"},
    {11, "Percussion"},
    {24, "Guitar"},
    {33, "Bass"},
    {40, "Violin"},
    {56, "Trumpet"},
    {90, "Pad"},
    {118, "Drums"},
};

/* Leds */
pin_t leds[NB_LEDS] = {
    {&DDRE, &PORTE, &PINE, 6},
    {&DDRF, &PORTF, &PINF, 0}
};

/* Button matrix lines */
pin_t lines[NB_LINES] = {
    {&DDRF, &PORTF, &PINF, 7},
    {&DDRF, &PORTF, &PINF, 6},
    {&DDRF, &PORTF, &PINF, 5},
    {&DDRF, &PORTF, &PINF, 4}
};

/* Button matrix columns */
pin_t cols[NB_COLS] = {
    {&DDRC, &PORTC, &PINC, 7},
    {&DDRC, &PORTC, &PINC, 6},
    {&DDRB, &PORTB, &PINB, 6},
    {&DDRB, &PORTB, &PINB, 5},
    {&DDRB, &PORTB, &PINB, 4},
};

/* Potentiometer */
pin_t pot = {&DDRF, &PORTF, &PINF, 1};

/* The last released and pressed buttons */
int isReleased = -1;
int isPressed = -1;

/* Indicates if there is a command to send */
bool toProcess = false;

/* Current octave, defaults to 3*/
int octave = 3;

/* Potentiometer volume */
uint8_t volume = 0;

/* Last sent volume */
uint8_t lastVolume = 0;

/* Filtered volume
 * This is a volume with a high pass filter applied
 * to reduce oscillations */
float filteredVolume = 0;

/* Array of button states, true if pressed */
bool buttonPressed[NB_LINES * NB_COLS] = {false};

/* Current instrument */
uint8_t currentInstrument = 0;

/* State of the buttons used to change mode */
ToggleModeState currentState = IDLE;

/* Current mode */
uint8_t mode = MODE_OCTAVE;

// 60 to 71 => Octave 3 in order C, C#, D, D#, E, F, F#, G, G#, A, A#, B
// 72 TO 77 => Octave 4 up to F
NoteMIDI notes[NB_NOTES] = { 
    {4, 60}, {3, 61}, {1, 62}, {2, 63}, {0, 64}, {9, 65},
    {8, 66}, {7, 67}, {6, 68}, {5, 69}, {14, 70}, {13, 71},
    {12, 72}, {11, 73}, {10, 74}, {19, 75}, {18, 76}, {17, 77}
};

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
    // Disable JTAG to use PF4 to PF7
    MCUCR |= (1<<JTD);
    MCUCR |= (1<<JTD);

    initializeClock();
    initializeIO();
    initializeLCD();
    
    ad_init(pot.bit);
    *pot.dir |= (1 << pot.bit);

    SetupHardware();

    LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
    GlobalInterruptEnable();

    for (;;)
    {
        scanKeyboard();
        toggleMode();    
        MIDI_Task();
        USB_USBTask();
    }
}