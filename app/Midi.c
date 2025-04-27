/*
 * MIDI-specific functionality
 */

#include "Midi.h"
#include "HD44780.h"

/* Updates the LCD and handles the octave up/down buttons 
 * This is done in a separate function because Endpoint_IsINReady() is false
 * when no software to read MIDI is open, so the LCD wouldn't be updated 
 * and octave wouldn't change */
void update_lcd(void)
{
    uint8_t lcdVolume;
    
    // Handle octave up/down button presses
    if (isReleased == BUTTON_OCTAVE_UP && toProcess && currentState != WAIT_RELEASE) {
        if (mode == MODE_OCTAVE) {
            octaveUp();
            HD44780_GoTo(31);
            HD44780_WriteInteger(octave, 10);
            isReleased = -1;
        } else if (mode == MODE_INSTRUMENT) {
            if (currentInstrument < NB_INSTRUMENTS - 1) {
                currentInstrument++;
            } else {
                currentInstrument = 0;
            }
            
            HD44780_GoTo(0);
            HD44780_WriteString("             ");
            HD44780_GoTo(0);
            HD44780_WriteString(instruments[currentInstrument].instrumentName);
            
            isReleased = -1;
        }
        toProcess = false;
    }
    else if (isReleased == BUTTON_OCTAVE_DOWN && toProcess && currentState != WAIT_RELEASE) {
        if (mode == MODE_OCTAVE) {
            octaveDown();
            HD44780_GoTo(31);
            HD44780_WriteInteger(octave, 10);
            isReleased = -1;
        } else if (mode == MODE_INSTRUMENT) {
            if (currentInstrument > 0) {
                currentInstrument--;
            } else {
                currentInstrument = NB_INSTRUMENTS - 1;
            }
            
            HD44780_GoTo(0);
            HD44780_WriteString("             ");
            HD44780_GoTo(0);
            HD44780_WriteString(instruments[currentInstrument].instrumentName);    

            isReleased = -1;
        }
        toProcess = false;
    }
        
    // Handle volume control
    uint8_t newReading = ad_capture();
    filteredVolume = (FILTER_WEIGHT * filteredVolume) + ((1 - FILTER_WEIGHT) * newReading); // Prevent the oscillation from sending too many messages
    uint8_t newVolume = (uint8_t)filteredVolume > MAX_POT ? MAX_POT : (uint8_t)filteredVolume;
    if (volume != newVolume) {
        volume = newVolume;
        lcdVolume = convert_volume(volume, MAX_VOLUME_LCD);
        
        // Update LCD with new volume regardless of MIDI state
        HD44780_GoTo(20);
        HD44780_WriteInteger(lcdVolume, 10);
        HD44780_WriteString("  ");
    }
}


/* Task to handle the generation of MIDI events and to send them to the host. */
void MIDI_Task(void)
{
    update_lcd();

    /* Device must be connected and configured for the task to run */
    if (USB_DeviceState != DEVICE_STATE_Configured)
    return;

    Endpoint_SelectEndpoint(MIDI_STREAM_IN_EPADDR);

    if (Endpoint_IsINReady())
    {
        uint8_t MIDICommand = 0;
        uint8_t MIDIPitch = 0;

        uint8_t midiVolume = convert_volume(volume, MAX_VOLUME_MIDI);

        if(isPressed != -1 && isPressed != BUTTON_OCTAVE_UP && isPressed != BUTTON_OCTAVE_DOWN && toProcess) {
            MIDICommand = MIDI_COMMAND_NOTE_ON;
            MIDIPitch = getMidiNote(isPressed);
            toProcess = false;
        }

        if(isReleased != -1 && isReleased != BUTTON_OCTAVE_UP && isReleased != BUTTON_OCTAVE_DOWN && toProcess) {
            MIDICommand = MIDI_COMMAND_NOTE_OFF;
            MIDIPitch = getMidiNote(isReleased);
            toProcess = false;
        }

        /* Check if the instrument has changed */
        if (mode == MODE_INSTRUMENT && 
            (isReleased == BUTTON_OCTAVE_UP || isReleased == BUTTON_OCTAVE_DOWN) && 
            !toProcess) {
            MIDICommand = MIDI_COMMAND_PROGRAM_CHANGE;
        }

        /* Check if the volume has changed and must be sent */
        if (volume != lastVolume) {
            MIDICommand = MIDI_COMMAND_CONTROL_CHANGE;
            lastVolume = volume;
        }

        /* Check if a MIDI command is to be sent */
        if (MIDICommand == MIDI_COMMAND_NOTE_ON || MIDICommand == MIDI_COMMAND_NOTE_OFF)
        {
            MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t)
                {
                    .Event       = MIDI_EVENT(0, MIDICommand),

                    .Data1       = MIDICommand | MIDI_CHANNEL(1),
                    .Data2       = MIDIPitch,
                    .Data3       = MIDI_STANDARD_VELOCITY,
                };

            /* Write the MIDI event packet to the endpoint */
            Endpoint_Write_Stream_LE(&MIDIEvent, sizeof(MIDIEvent), NULL);

            /* Send the data in the endpoint to the host */
            Endpoint_ClearIN();
        }
        else if (MIDICommand == MIDI_COMMAND_CONTROL_CHANGE) {
            MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t)
                {
                    .Event    = (MIDI_EVENT(0, MIDICommand)),

                    .Data1    = MIDICommand | MIDI_CHANNEL(1),
                    .Data2    = 14,
                    .Data3    = midiVolume
                };

            Endpoint_Write_Stream_LE(&MIDIEvent, sizeof(MIDIEvent), NULL);
            Endpoint_ClearIN();
        } else if (MIDICommand == MIDI_COMMAND_PROGRAM_CHANGE) {
            MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t)
                {
                    .Event    = (MIDI_EVENT(0, MIDICommand)),
                    .Data1    = MIDICommand | MIDI_CHANNEL(1),
                    .Data2    = instruments[currentInstrument].instrument,
                    .Data3    = 0
                };

            Endpoint_Write_Stream_LE(&MIDIEvent, sizeof(MIDIEvent), NULL);
            Endpoint_ClearIN();
        }

    }

    /* Select the MIDI OUT stream */
    Endpoint_SelectEndpoint(MIDI_STREAM_OUT_EPADDR);

    /* Check if a MIDI command has been received */
    if (Endpoint_IsOUTReceived())
    {
        MIDI_EventPacket_t MIDIEvent;

        /* Read the MIDI event packet from the endpoint */
        Endpoint_Read_Stream_LE(&MIDIEvent, sizeof(MIDIEvent), NULL);

        /* Check to see if the sent command is a note on message with a non-zero velocity */
        if ((MIDIEvent.Event == MIDI_EVENT(0, MIDI_COMMAND_NOTE_ON)) && (MIDIEvent.Data3 > 0))
        {
            /* Change LEDs depending on the pitch of the sent note */
            LEDs_SetAllLEDs(MIDIEvent.Data2 > 64 ? LEDS_LED1 : LEDS_LED2);
        }
        else
        {
            /* Turn off all LEDs in response to non Note On messages */
            LEDs_SetAllLEDs(LEDS_NO_LEDS);
        }

        /* If the endpoint is now empty, clear the bank */
        if (!(Endpoint_BytesInEndpoint()))
        {
            /* Clear the endpoint ready for new packet */
            Endpoint_ClearOUT();
        }
    }
}

/* Get the MIDI note corresponding to a button */
int getMidiNote(int buttonId) {
    for (int i = 0; i < NB_NOTES; i++) {
        if (buttonId == notes[i].button) {
            return notes[i].midiNote;
        }
    }

    return -1;
}

/* Increase the octave */
void octaveUp(void) {
    for (int i = 0; i < NB_NOTES; i++) {
        notes[i].midiNote += NB_NOTES;
    }
    if (octave < MAX_OCTAVE) {
        octave++;
    }
}

/* Decrease the octave */
void octaveDown(void) {
    for (int i = 0; i < NB_NOTES; i++) {
        notes[i].midiNote -= NB_NOTES;
    }
    if (octave > MIN_OCTAVE) {
        octave--;
    }
}