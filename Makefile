export CC = avr-gcc
export LD = avr-gcc
export MCU = atmega32u4
export FCPU = 16000000
export FLAGS = -mmcu=$(MCU)
export CFLAGS = -Wall $(FLAGS) -DF_CPU=$(FCPU) -Os
export LDFLAGS = $(FLAGS)

export PROGRAMMER = dfu-programmer

TARGET = app
SOURCES = $(wildcard *.c)
OBJECTS = $(SOURCES:.c=.o)

all: $(TARGET).hex

clean:
	rm -f *.o $(TARGET).hex $(TARGET)

$(TARGET): $(OBJECTS)

$(TARGET).hex: $(TARGET)
	avr-objcopy -j .text -j .data -O ihex $(TARGET) $(TARGET).hex

upload: $(TARGET).hex
	$(PROGRAMMER) $(MCU) erase
	$(PROGRAMMER) $(MCU) flash --debug 2 $(TARGET).hex
	$(PROGRAMMER) $(MCU) reset

size: $(TARGET)
	avr-size --format=avr --mcu=$(MCU) $(TARGET)