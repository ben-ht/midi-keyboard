/*
 * Keyboard functions
 */

#include "Keyboard.h"

/* Scan the keyboard to determine which key is pressed */
void scanKeyboard(void) {
    for(uint8_t l = 0; l < NB_LINES ; l++) {

        *lines[l].port &= ~(1 << lines[l].bit);

        for(uint8_t c = 0; c < NB_COLS; c++) {

            int t = l * NB_COLS + c;

            if(!(*cols[c].pin & (1 << cols[c].bit))) {
                if(!buttonPressed[t]){
                    buttonPressed[t] = true;
                    isPressed = t;
                    isReleased = -1;
                    toProcess = true;
                }
            } else {
                if(buttonPressed[t]){
                    buttonPressed[t] = false;
                    isReleased = t;
                    isPressed = -1;
                    toProcess = true;
                }             
            }    
        }

        *lines[l].port |= (1 << lines[l].bit);
    }
}

/* Check if a mode change is requested */
bool toggleModeRequested(void) {
    return buttonPressed[BUTTON_OCTAVE_UP] && buttonPressed[BUTTON_OCTAVE_DOWN];
}

/* Check if a mode change is allowed */
bool toggleModeAllowed(void) {
    return !buttonPressed[BUTTON_OCTAVE_UP] && !buttonPressed[BUTTON_OCTAVE_DOWN];
}

/* Toggle the mode
 * IDLE: Wait for a mode change request
 * WAIT_RELEASE: Wait for the release of the button
 * Mode is toggled on release in order not to change an octave or instrument on press
 */
void toggleMode() {
    switch (currentState) {
        case IDLE:
            if (toggleModeRequested()) {
                currentState = WAIT_RELEASE;
            }
            break;
        case WAIT_RELEASE:
            if (toggleModeAllowed()) {
                mode = !mode;

                if (mode) {
                    HD44780_GoTo(13);
                    HD44780_WriteString("INS");
                } else {
                    HD44780_GoTo(13);
                    HD44780_WriteString("OCT");
                }

                currentState = IDLE;
                isReleased = -1; //Prevent octave change on release
            }
            break;
    }
}