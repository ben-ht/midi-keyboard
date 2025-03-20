/*
			LUFA Library
	Copyright (C) Dean Camera, 2021.

dean [at] fourwalledcubicle [dot] com
		www.lufa-lib.org
*/

/*
Copyright 2021  Dean Camera (dean [at] fourwalledcubicle [dot] com)

Permission to use, copy, modify, distribute, and sell this
software and its documentation for any purpose is hereby granted
without fee, provided that the above copyright notice appear in
all copies and that both that the copyright notice and this
permission notice and warranty disclaimer appear in supporting
documentation, and that the name of the author not be used in
advertising or publicity pertaining to distribution of the
software without specific, written prior permission.

The author disclaims all warranties with regard to this
software, including all implied warranties of merchantability
and fitness.  In no event shall the author be liable for any
special, indirect or consequential damages or any damages
whatsoever resulting from loss of use, data or profits, whether
in an action of contract, negligence or other tortious action,
arising out of or in connection with the use or performance of
this software.
*/

/** \file
*
*  Main source file for the MIDI input demo. This file contains the main tasks of the demo and
*  is responsible for the initial application hardware configuration.
*/

#include "MIDI.h"
#include "HD44780.h"

#define NB_LEDS		2
#define NB_LINES	4
#define NB_COLS		5
#define ADFR		5
#define FILTER_WEIGHT	0.8
#define MAX_VOLUME_LCD	100
#define MAX_VOLUME_MIDI	127
#define MAX_POT		38

typedef struct {
	volatile unsigned char *dir;
	volatile unsigned char *port;
	volatile unsigned char *pin;
	unsigned char bit;
} pin_t;

typedef struct {
	int keysButton;
	unsigned midiNote;
} NoteMIDI;

pin_t leds[NB_LEDS]={
{&DDRE,&PORTE,&PINE,6},
{&DDRF,&PORTF,&PINF,0}
};

pin_t lines[NB_LINES]={
{&DDRF,&PORTF,&PINF,7},
{&DDRF,&PORTF,&PINF,6},
{&DDRF,&PORTF,&PINF,5},
{&DDRF,&PORTF,&PINF,4}
};

pin_t cols[NB_COLS]={
{&DDRC,&PORTC,&PINC,7},
{&DDRC,&PORTC,&PINC,6},
{&DDRB,&PORTB,&PINB,6},
{&DDRB,&PORTB,&PINB,5},
{&DDRB,&PORTB,&PINB,4},
};

pin_t pot = {&DDRF,&PORTF,&PINF,1};

int isReleased = -1;
int isPressed = -1;
bool toProcess = false;
int octave = 3; // Defaults to 3rd octave
uint8_t volume = 0;
float filteredVolume = 0;

// 60 to 71 => Octave 3 in order C, C#, D, D#, E, F, F#, G, G#, A, A#, B
// 72 TO 77 => Octave 4 up to F
NoteMIDI notes[18] = { 
	{4, 60}, {3, 61}, {1, 62}, {2, 63}, {0, 64}, {9, 65},
	{8, 66}, {7, 67}, {6, 68}, {5, 69}, {14, 70}, {13, 71},
	{12, 72}, {11, 73}, {10, 74}, {19, 75}, {18, 76}, {17, 77}
};

int giveMidiNote(int);
void scanne_touches(void);
void conf_horloge(void);
void init(void);
void octaveUp(void);
void octaveDown(void);
void handleOctaveLeds(void);
void octaveTask(void);
void ad_init(unsigned char channel);
unsigned int ad_capture(void);
uint8_t convert_volume(uint8_t volume, uint8_t maxVolume);

/** Main program entry point. This routine configures the hardware required by the application, then
*  enters a loop to run the application tasks in sequence.
*/
int main(void)
{
	conf_horloge();

	// désactive JTAG pour utilisation de PF4 à PF7 :
	MCUCR |= (1<<JTD);
	MCUCR |= (1<<JTD);

	init();

	HD44780_Initialize();
	HD44780_WriteCommand(LCD_ON|CURSOR_NONE);
	HD44780_WriteCommand(LCD_CLEAR);
	HD44780_WriteCommand(LCD_HOME);
	HD44780_WriteCommand(LCD_INCR_RIGHT);

	_delay_ms(500);

	HD44780_WriteString("MIDI Keyboard");
	HD44780_GoTo(16);
	HD44780_WriteString("Vol:");
	HD44780_WriteInteger(0, 10);
	HD44780_GoTo(27);
	HD44780_WriteString("Oct:");
	HD44780_WriteInteger(octave, 10);
	
	ad_init(pot.bit);
	*pot.dir |= (1 << pot.bit);

	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;)
	{
		scanne_touches();
		octaveTask();
		MIDI_Task();
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	Joystick_Init();
	LEDs_Init();
	Buttons_Init();
	USB_Init();
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs. */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
*  the status LEDs, disables the sample update and PWM output timers and stops the USB and MIDI management tasks.
*/
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
*  of the USB device after enumeration - the device endpoints are configured and the MIDI management task started.
*/
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup MIDI Data Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(MIDI_STREAM_IN_EPADDR, EP_TYPE_BULK, MIDI_STREAM_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(MIDI_STREAM_OUT_EPADDR, EP_TYPE_BULK, MIDI_STREAM_EPSIZE, 1);

	/* Indicate endpoint configuration success or failure */
	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Task to handle the generation of MIDI note change events in response to presses of the board joystick, and send them
*  to the host.
*/
void MIDI_Task(void)
{
	static uint8_t PrevJoystickStatus;

	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	return;

	Endpoint_SelectEndpoint(MIDI_STREAM_IN_EPADDR);

	if (Endpoint_IsINReady())
	{
		uint8_t MIDICommand = 0;
		uint8_t MIDIPitch;

		uint8_t lcdVolume;
		uint8_t midiVolume;

		uint8_t JoystickStatus  = Joystick_GetStatus();
		uint8_t JoystickChanges = (JoystickStatus ^ PrevJoystickStatus);

		/* Get board button status - if pressed use channel 10 (percussion), otherwise use channel 1 */
		uint8_t Channel = ((Buttons_GetStatus() & BUTTONS_BUTTON1) ? MIDI_CHANNEL(10) : MIDI_CHANNEL(1));

		if(isPressed != -1 && toProcess) {
			MIDICommand = MIDI_COMMAND_NOTE_ON;
			MIDIPitch = giveMidiNote(isPressed);
			toProcess = false;
		}

		if(isReleased != -1 && toProcess) {
			MIDICommand = MIDI_COMMAND_NOTE_OFF;
			MIDIPitch = giveMidiNote(isReleased);
			toProcess = false;
		}

		uint8_t newReading = ad_capture();
		filteredVolume = (FILTER_WEIGHT * filteredVolume) + ((1 - FILTER_WEIGHT) * newReading); // Prevent the oscillation from sending too many messages
		uint8_t newVolume = (uint8_t)filteredVolume;
		if (volume != newVolume) {
			volume = newVolume;
			lcdVolume = convert_volume(volume, MAX_VOLUME_LCD);
			midiVolume = convert_volume(volume, MAX_VOLUME_MIDI);
			if (lcdVolume > 100) { //Volume sometimes reaches 102
				lcdVolume = 100;
			}
			MIDICommand = MIDI_COMMAND_CONTROL_CHANGE;
			HD44780_GoTo(20);
			HD44780_WriteInteger(lcdVolume, 10);
			HD44780_WriteString("   ");
		} 

		/* Check if a MIDI command is to be sent */
		if (MIDICommand == MIDI_COMMAND_NOTE_ON || MIDICommand == MIDI_COMMAND_NOTE_OFF)
		{
			MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t)
				{
					.Event       = MIDI_EVENT(0, MIDICommand),

					.Data1       = MIDICommand | Channel,
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
					.Event	= (MIDI_EVENT(0, MIDICommand)),

					.Data1	= MIDICommand | Channel,
					.Data2	= 7,
					.Data3	= midiVolume
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


void scanne_touches(void) {
	for(uint8_t l=0; l<NB_LINES ; l++) {

		// activer une ligne à la fois
		*lines[l].port &= ~(1 << lines[l].bit); // Cmet la ligne à 0V

		// vérifier chaque colonne
		for(uint8_t c = 0; c < NB_COLS; c++) {

			int t = l * NB_COLS + c;

			if(!(*cols[c].pin & (1 << cols[c].bit))) { // Si une colonne est à 0V
				// touche enfoncée
				if(isPressed == -1){
					isPressed = t;
					isReleased = -1;
					toProcess = true;
				}
			} else {
				// pas appuyée
				if(isPressed == t){
					isReleased = t;
					isPressed = -1;
					toProcess = true;
				} 
			}		
		}

		*lines[l].port |= (1 << lines[l].bit);
	}
}

void conf_horloge(void){
	CLKSEL0 = 0b00010101;   // sélection de l'horloge externe
	CLKSEL1 = 0b00010000;   // minimum de 16Mhz
	CLKPR = 0b10000000;     // modification du diviseur d'horloge (CLKPCE=1)
	CLKPR = 0;              // 0 pour pas de diviseur (diviseur de 1)
}

void init(void){
	// Set LEDs as output
	for(int i=0;i<NB_LEDS;i++) *leds[i].dir |= (1 << leds[i].bit);

	// Set cols as input + pull up charge
	for(int i=0;i<NB_COLS;i++){
		*cols[i].dir &= ~(1 << cols[i].bit); // 0 : une entrée
		*cols[i].port |= (1 << cols[i].bit); // 1 : pull up, résist charge
	}

	// Set rows as ouput
	for(int i=0;i<NB_LINES;i++) *lines[i].dir |= (1 << lines[i].bit);
}

int giveMidiNote(int buttonId) {
	// Recherche d'une note
	int taille = sizeof(notes) / sizeof(notes[0]);
	for (int i = 0; i < taille; i++) {
		if (buttonId == notes[i].keysButton) {
			return notes[i].midiNote;
		}
	}

	return -1;
}

void octaveUp(void) {
	size_t length = sizeof(notes) / sizeof(notes[0]);
	for (int i = 0; i < length; i++) {
		notes[i].midiNote += 18;
	}
	if (octave < 7) {
		octave++;
	}
}


void octaveDown(void) {
	size_t length = sizeof(notes) / sizeof(notes[0]);
	for (int i = 0; i < length; i++) {
		notes[i].midiNote -= 18;
	}
	if (octave > 1) {
		octave--;
	}
}


void handleOctaveLeds(void) {
	if (octave > 3) {
		*leds[0].port |= (1 << leds[0].bit);
	} else if (octave < 3) {
		*leds[1].port |= (1 << leds[1].bit);
	} else {
		*leds[0].port &= ~(1 << leds[0].bit);
		*leds[1].port &= ~(1 << leds[1].bit);
	}
}

void octaveTask(void) {
	if (isPressed == 16 && toProcess) {
		octaveUp();
		handleOctaveLeds();
		HD44780_GoTo(31);
		HD44780_WriteInteger(octave, 10);
		isPressed = -1;
		toProcess = false;
		_delay_ms(250);
	} else if (isPressed == 15 && toProcess) {
		octaveDown();
		handleOctaveLeds();
		HD44780_GoTo(31);
		HD44780_WriteInteger(octave, 10);
		isPressed = -1;
		toProcess = false;
		_delay_ms(250);
	}
}
void ad_init(unsigned char channel){
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Division frequency 128 => 125KHz
    ADCSRA &= ~(1<<ADFR);                       // Single conversion mode
    ADMUX |= (1<<REFS0)|(1<<ADLAR);             // Voltage reference AVCC, left-adjust result
    ADMUX = (ADMUX & 0xf0) | channel;           // Select channel   
    ADCSRA |= (1<<ADEN);                        // Enable ADC
}

unsigned int ad_capture(void){
    ADCSRA |= (1<<ADSC);                      // Start conversion
    while(bit_is_set(ADCSRA, ADSC));          // Wait for conversion to complete
    return ADCH;                              // Return 8-bit result (0-255)
}


uint8_t convert_volume(uint8_t volume, uint8_t maxVolume) {
	return (volume * maxVolume) / MAX_POT; 
}
