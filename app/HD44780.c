/*
 * LCD Screen functions
 */

#include "HD44780.h"

static void HD44780_WriteNibble(const uint8_t nib)
{
  LCD_EN_PORT &= ~(1<<LCD_EN_PIN);
  if(nib&0x08) LCD_D7_PORT |= (1<<LCD_D7_PIN);
  else LCD_D7_PORT &= ~(1<<LCD_D7_PIN);
  if(nib&0x04) LCD_D6_PORT |= (1<<LCD_D6_PIN);
  else LCD_D6_PORT &= ~(1<<LCD_D6_PIN);
  if(nib&0x02) LCD_D5_PORT |= (1<<LCD_D5_PIN);
  else LCD_D5_PORT &= ~(1<<LCD_D5_PIN);
  if(nib&0x01) LCD_D4_PORT |= (1<<LCD_D4_PIN);
  else LCD_D4_PORT &= ~(1<<LCD_D4_PIN);
  _delay_us(1);
  LCD_EN_PORT |= (1<<LCD_EN_PIN);
  _delay_us(1);
  LCD_EN_PORT &= ~(1<<LCD_EN_PIN);
  _delay_us(100);
}

static void HD44780_WriteByte(const uint8_t c)
{
  HD44780_WriteNibble(c>>4);
  HD44780_WriteNibble(c&0x0F);
}

static void HD44780_PowerUp4Bit(void)
{
  /* Wait for more than 40 ms after VCC rises to 2.7 V */
  _delay_ms(40);
  HD44780_WriteNibble(0x03);        // FN_SET 8-bit

  /* Wait for more than 4.1 ms */
  _delay_ms(5);
  HD44780_WriteNibble(0x03);        // FN_SET 8-bit

  /* Wait for more than 100 µs */
  _delay_us(100);
  HD44780_WriteNibble(0x03);        // FN_SET 8-bit

  /* From now on we must allow 40us for each command */
  _delay_us(50);
  HD44780_WriteNibble(0x02);        // FN_SET 4-bit

  /* The LCD is now in 4-bit mode so we can continue
     using the 4-bit API */
  _delay_us(50);
}

void HD44780_Initialize(void)
{
  LCD_D4_DDR |= (1<<LCD_D4_PIN);
  LCD_D5_DDR |= (1<<LCD_D5_PIN);
  LCD_D6_DDR |= (1<<LCD_D6_PIN);
  LCD_D7_DDR |= (1<<LCD_D7_PIN);
  LCD_RS_DDR |= (1<<LCD_RS_PIN);
  LCD_RW_DDR |= (1<<LCD_RW_PIN);
  LCD_EN_DDR |= (1<<LCD_EN_PIN);
  LCD_D4_PORT &= ~(1<<LCD_D4_PIN);
  LCD_D5_PORT &= ~(1<<LCD_D5_PIN);
  LCD_D6_PORT &= ~(1<<LCD_D6_PIN);
  LCD_D7_PORT &= ~(1<<LCD_D7_PIN);
  LCD_RS_PORT &= ~(1<<LCD_RS_PIN);
  LCD_RW_PORT &= ~(1<<LCD_RW_PIN);
  LCD_EN_PORT &= ~(1<<LCD_EN_PIN);
  HD44780_PowerUp4Bit();
  _delay_ms(50);
}

void HD44780_WriteCommand(const uint8_t c)
{
  LCD_RS_PORT &= ~(1<<LCD_RS_PIN);
  HD44780_WriteByte(c);
  _delay_us(50);
}

void HD44780_WriteData(const uint8_t c)
{
  LCD_RS_PORT |= (1<<LCD_RS_PIN);
  HD44780_WriteByte(c);
  LCD_RS_PORT &= ~(1<<LCD_RS_PIN);
  _delay_us(50);
}

void HD44780_WriteString(char *string)
{
  for(int i=0;i<strlen(string);i++) HD44780_WriteData(string[i]);
}

#define	MAX_DIGITS	32
void HD44780_WriteInteger(int num,int radix){
char s[MAX_DIGITS];
itoa(num,s,radix);
HD44780_WriteString(s);
}

int HD44780_XY2Adrr(int nbrows,int nbcols,int row,int col)
{
  int row_offsets[]={0x00,0x40,0x14,0x54};
  if(row>=nbrows) row=nbrows-1;
  if(col>=nbcols) col=nbcols-1;
  return row_offsets[row]+col;
}



/* Move carret to a specific position */
void HD44780_GoTo(unsigned char pos) {
	if (pos < 16) {
		HD44780_WriteCommand(0x80 + pos);
	} else {
		HD44780_WriteCommand(0xC0 + (pos - 16));
	}
}

/* Displays MIDI Keyboard characters one by one */
void playStartupAnimation(void) {
    const char* text = "MIDI Keyboard";
    int text_length = strlen(text);
    char display[17]; // 16 characters + null terminator
    
    memset(display, ' ', 16);
    display[16] = '\0';
    
    HD44780_GoTo(0);
    HD44780_WriteString(display);
    _delay_ms(200);
    
    for (int i = 0; i < text_length; i++) {
        int start_pos = (16 - text_length) / 2;
        
        for (int j = 0; j <= i; j++) {
            display[start_pos + j] = text[j];
        }
        
        HD44780_GoTo(0);
        HD44780_WriteString(display);
        _delay_ms(150);
    }
    
    _delay_ms(500);
}

/* Startup sequence of the LCD screen */
void initializeLCD(void) {
	HD44780_Initialize();
	HD44780_WriteCommand(LCD_ON|CURSOR_NONE);
	HD44780_WriteCommand(LCD_CLEAR);
	HD44780_WriteCommand(LCD_HOME);
	HD44780_WriteCommand(LCD_INCR_RIGHT);

	_delay_ms(150);
	playStartupAnimation();
	HD44780_WriteCommand(LCD_CLEAR);
	_delay_ms(150);

	HD44780_GoTo(0);
	HD44780_WriteString(instruments[currentInstrument].instrumentName);
	HD44780_GoTo(13);
	HD44780_WriteString("OCT");
	HD44780_GoTo(16);
	HD44780_WriteString("Vol:");
	HD44780_WriteInteger(0, 10);
	HD44780_GoTo(27);
	HD44780_WriteString("Oct:");
	HD44780_WriteInteger(octave, 10);
}