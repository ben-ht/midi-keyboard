#
#             LUFA Library
#     Copyright (C) Dean Camera, 2021.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.

MCU          = atmega32u4
ARCH         = AVR8
BOARD        = USBKEY
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = main
SRC          = $(TARGET).c HD44780.c Descriptors.c Hardware.c Keyboard.c Midi.c $(LUFA_SRC_USB)
LUFA_PATH    = ../LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/
LD_FLAGS     =
PROGRAMMER	 = dfu-programmer
FLAGS = -mmcu=$(MCU)
CFLAGS = -Wall $(FLAGS) -DF_CPU=$(F_CPU) -Os
LDFLAGS = $(FLAGS)
OBJECTS = $(SRC:.c=.o)

# Default target
all: $(TARGET).hex

# Include LUFA-specific DMBS extension modules
DMBS_LUFA_PATH ?= $(LUFA_PATH)/Build/LUFA
include $(DMBS_LUFA_PATH)/lufa-sources.mk
include $(DMBS_LUFA_PATH)/lufa-gcc.mk

# Include common DMBS build system modules
DMBS_PATH ?= $(LUFA_PATH)/Build/DMBS/DMBS
include $(DMBS_PATH)/core.mk
include $(DMBS_PATH)/cppcheck.mk
include $(DMBS_PATH)/doxygen.mk
include $(DMBS_PATH)/dfu.mk
include $(DMBS_PATH)/gcc.mk
include $(DMBS_PATH)/hid.mk
include $(DMBS_PATH)/avrdude.mk
include $(DMBS_PATH)/atprogram.mk

upload: $(TARGET).hex
	$(PROGRAMMER) $(MCU) erase
	$(PROGRAMMER) $(MCU) flash --debug 2 $(TARGET).hex
	$(PROGRAMMER) $(MCU) reset

clean:
	rm -f *.o *.eep *.bin *.elf *.lss *.map *.sym *.txt $(TARGET).hex $(TARGET)