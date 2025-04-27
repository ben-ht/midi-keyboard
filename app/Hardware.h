/*
 * Hardware initialization functions and ADC functions
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_

#include "Keyboard.h"

#include <util/delay.h>
#include <avr/power.h>

#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Board/LEDs.h>

 /* Macros: */
/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
#define LEDMASK_USB_NOTREADY      LEDS_LED1

/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
#define LEDMASK_USB_ENUMERATING  (LEDS_LED2 | LEDS_LED3)

/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
#define LEDMASK_USB_READY        (LEDS_LED2 | LEDS_LED4)

/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
#define LEDMASK_USB_ERROR        (LEDS_LED1 | LEDS_LED3)

// Function declarations
void SetupHardware(void);
void initializeClock(void);
void initializeIO(void);
void ad_init(unsigned char channel);
unsigned int ad_capture(void);
uint8_t convert_volume(uint8_t volume, uint8_t maxVolume);
void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);


#endif // _HARDWARE_H_