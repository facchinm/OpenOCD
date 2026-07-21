BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS =	-c -mthumb -mcpu=cortex-m0plus -Os -Wall -Wextra -Wpedantic \
		-fpic -mpic-data-is-text-relative -ffreestanding -fwhole-program

all: psoc4_write.inc psoc4hv_write.inc

.PHONY: clean

.INTERMEDIATE: psoc4_write.o psoc4hv_write.o

psoc4_write.o: psoc4hv_write.c Makefile
	$(CC) $(CFLAGS) -DPSOC4_VARIANT_PSOC4 -Wa,-adhln=$*.lst $< -o $@
	
psoc4hv_write.o: psoc4hv_write.c Makefile
	$(CC) $(CFLAGS) -DPSOC4_VARIANT_PSOC4HV -Wa,-adhln=$*.lst $< -o $@

%.bin: %.o
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.o *.lst *.bin *.inc
