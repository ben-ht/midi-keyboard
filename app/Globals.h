#ifndef _GLOBALS_H_
#define _GLOBALS_H_

/* Number of leds */
#define NB_LEDS		2

/* Number of rows and columns in the button matrix */
#define NB_LINES	4
#define NB_COLS		5

/* ADC channel */
#define ADFR		5

/* Volume filter */
#define FILTER_WEIGHT	0.8

/* Maximum volume to display on the LCD */
#define MAX_VOLUME_LCD	100

/* Maximum volume to send to the host, volume is on 8 bits */
#define MAX_VOLUME_MIDI	127

/* Maximum potentiometer value */
#define MAX_POT		38

/* Modes
 * Octave mode enables to increase or decrease the octave
 * Instrument mode enables to change the instrument 
 */
#define MODE_OCTAVE	0
#define MODE_INSTRUMENT	1

/* Number of instruments */
#define NB_INSTRUMENTS	8

/* Number of notes / note buttons on the keyboard */
#define NB_NOTES	18

/* Buttons to change the octave or instrument */
#define BUTTON_OCTAVE_UP	16
#define BUTTON_OCTAVE_DOWN	15

/* Octave range */
#define MIN_OCTAVE 1
#define MAX_OCTAVE 7

/* Hardware pins mapping */
typedef struct {
    volatile uint8_t *dir;
    volatile uint8_t *port;
    volatile uint8_t *pin;
    uint8_t bit;
} pin_t;

/* Instrument structure */
typedef struct {
    uint8_t instrument;
    char *instrumentName;
} Instrument;

/* MIDI Note structure */
typedef struct {
    uint8_t button;
    uint8_t midiNote;
} NoteMIDI;

/* Toggle mode button state */
typedef enum {
    IDLE,
    WAIT_RELEASE
} ToggleModeState;

// Global variables declarations (defined in MIDI_Keyboard.c)
extern Instrument instruments[NB_INSTRUMENTS];
extern pin_t leds[NB_LEDS];
extern pin_t lines[NB_LINES];
extern pin_t cols[NB_COLS];
extern pin_t pot;
extern int isReleased;
extern int isPressed;
extern bool toProcess;
extern int octave;
extern uint8_t volume;
extern uint8_t lastVolume;
extern float filteredVolume;
extern bool buttonPressed[NB_LINES * NB_COLS];
extern uint8_t currentInstrument;
extern ToggleModeState currentState;
extern uint8_t mode;
extern NoteMIDI notes[NB_NOTES];

#endif // _GLOBALS_H_