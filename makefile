#
#             LUFA Library
#     Copyright (C) Dean Camera, 2014.
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
BOARD        = LEONARDO
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = sx_filter
SRC          = $(TARGET).c Descriptors.c stepper.c LEDS.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS_DEVICE)
LUFA_PATH    = ../../LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/
LD_FLAGS     =

# Specifically for the Arduino Due
CC_FLAGS += -DAVR_RESET_LINE_PORT="PORTC"
CC_FLAGS += -DAVR_RESET_LINE_DDR="DDRC"
CC_FLAGS += -DAVR_RESET_LINE_MASK="(1 << 7)"
CC_FLAGS += -DAVR_ERASE_LINE_PORT="PORTC"
CC_FLAGS += -DAVR_ERASE_LINE_DDR="DDRC"
CC_FLAGS += -DAVR_ERASE_LINE_MASK="(1 << 6)"

# Optional Variables
AVRDUDE_PROGRAMMER = atmelice_isp
ARDUINO_MODEL_PID = 0x0010

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk
