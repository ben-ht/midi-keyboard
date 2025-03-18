// Commands
#define LCD_CLEAR	0x01
#define LCD_HOME	0x02
#define LCD_ON		0x0c
#define LCD_OFF		0x08
#define LCD_INCR_LEFT	0x04
#define LCD_INCR_RIGHT	0x06
#define LCD_ADDRSET	0x80

// Scroll mode
#define SCROLL		0x01

// Cursor mode
#define CURSOR_NONE	0x00
#define CURSOR_BLINK	0x01
#define CURSOR_ULINE	0x02
#define CURSOR_FULL	0x03

// Prototypes
void HD44780_Initialize(void);
void HD44780_WriteCommand(const uint8_t c);
void HD44780_WriteData(const uint8_t c);
void HD44780_WriteString(char *string);
void HD44780_WriteInteger(int num,int radix);
int HD44780_XY2Adrr(int nbrows,int nbcols,int row,int col);
void HD44780_GoTo(unsigned char pos);
