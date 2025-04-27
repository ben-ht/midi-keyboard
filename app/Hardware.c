/*
 * Hardware initialization functions and ADC functions
 */

#include "hardware.h"

#include <avr/wdt.h>

/* Initialize the clock */
void initializeClock(void){
    CLKSEL0 = 0b00010101;   // sélection de l'horloge externe
    CLKSEL1 = 0b00010000;   // minimum de 16Mhz
    CLKPR = 0b10000000;     // modification du diviseur d'horloge (CLKPCE=1)
    CLKPR = 0;              // 0 pour pas de diviseur (diviseur de 1)
}

/* Initialize buttons and LEDs */
void initializeIO(void){
    // Set LEDs as output
    for(int i=0;i<NB_LEDS;i++) {
        *leds[i].dir |= (1 << leds[i].bit);
    }

    // Set cols as input + pull up charge
    for(int i=0;i<NB_COLS;i++){
        *cols[i].dir &= ~(1 << cols[i].bit); // input
        *cols[i].port |= (1 << cols[i].bit); // pull up charge
    }

    // Set rows as ouput
    for(int i=0;i<NB_LINES;i++) {
        *lines[i].dir |= (1 << lines[i].bit);
    }
}

/* Initialize the ADC */
void ad_init(unsigned char channel){
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Division frequency 128 => 125KHz
    ADCSRA &= ~(1<<ADFR);                       // Single conversion mode
    ADMUX |= (1<<REFS0)|(1<<ADLAR);             // Voltage reference AVCC, left-adjust result
    ADMUX = (ADMUX & 0xf0) | channel;           // Select channel   
    ADCSRA |= (1<<ADEN);                        // Enable ADC
}

/* Capture the ADC value */
unsigned int ad_capture(void){
    ADCSRA |= (1<<ADSC);                      // Start conversion
    while(bit_is_set(ADCSRA, ADSC));          // Wait for conversion to complete
    return ADCH;                              // Return 8-bit result (0-255)
}

/* Convert the ADC value to a volume */
uint8_t convert_volume(uint8_t volume, uint8_t maxVolume) {
    return (volume * maxVolume) / MAX_POT; 
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
    LEDs_Init();
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