#include <avr/io.h>
#include <util/delay.h>
#include "HD44780.h"

typedef struct {
  volatile unsigned char *dir;
  volatile unsigned char *port;
  volatile unsigned char *pin;
  unsigned char bit;
} pin_t;

#define NB_LEDS		2
#define NB_LINES	4
#define NB_COLS		5

#define ADFR      5

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

pin_t pot = {&DDRF, &PORTF, &PINF, 1};

void conf_horloge();
void init();
void clignote();
void ad_init(unsigned char channel);
unsigned int ad_capture(void);

int main(void){
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
    HD44780_WriteString("MIDI KEYBOARD");

    ad_init(pot.bit);
    *pot.dir |= (1 << pot.bit);
    unsigned int pot_value;

    for(int i=0;i<NB_LINES;i++) *lines[i].port |= (1 << lines[i].bit);

    while(1){
        pot_value = ad_capture();
        HDD44780_GoTo(28);
        HD44780_WriteInteger(pot_value, 10);
        HD44780_WriteString("   ");
        _delay_ms(200);

        for(uint8_t l=0 ; l<NB_LINES ; l++) {

            // activer une ligne à la fois
            *lines[l].port &= ~(1 << lines[l].bit) ; // Cmet la ligne à 0V

            // vérifier chaque colonne
            for(uint8_t c = 0; c < NB_COLS; c++) {

                int t = l * NB_COLS + c ;

                if(!(*cols[c].pin & (1 << cols[c].bit))) { // Si une colonne est à 0V
                    if(t == 15) clignote() ;
                }
                
            }

            *lines[l].port |= (1 << lines[l].bit) ;
        }
        
    }
}

void clignote() {
    for (uint8_t i = 0; i < 3; i++){
        *leds[0].port |= (1 << leds[0].bit); // Turn LED on
        _delay_ms(500);
        *leds[0].port &= ~(1 << leds[0].bit); // Turn LED off
        *leds[1].port |= (1 << leds[1].bit);
        _delay_ms(500);
        *leds[1].port &= ~(1 << leds[1].bit);
    }
}

void conf_horloge(){
    CLKSEL0 = 0b00010101;   // sélection de l'horloge externe
    CLKSEL1 = 0b00010000;   // minimum de 16Mhz
    CLKPR = 0b10000000;     // modification du diviseur d'horloge (CLKPCE=1)
    CLKPR = 0;              // 0 pour pas de diviseur (diviseur de 1)
}

void init(){
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