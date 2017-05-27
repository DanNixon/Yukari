V = 1

BINARY = main
OBJS += $(BINARY).o clock.o console.o mpu6000.o MadgwickAHRS.o leds.o

DEVICE = STM32F405RG

################################################################################
# OpenOCD specific variables

OOCD ?= openocd
OOCD_INTERFACE ?= stlink-v2
OOCD_TARGET ?= stm32f4x

################################################################################
# Black Magic Probe specific variables
# Set the BMP_PORT to a serial port and then BMP is used for flashing
BMP_PORT ?=

################################################################################
# texane/stlink specific variables
# STLINK_PORT ?= :4242


include rules.mk
