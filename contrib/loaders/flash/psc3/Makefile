BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS =  -c -mthumb -mcpu=cortex-m33 -Os
CFLAGS += -Wall -Wextra

all: psc3_write.inc psc3_erase.inc

.PHONY: clean

.INTERMEDIATE: psc3_write.o psc3_erase.o

psc3_write.o: psc3_write.c
	$(CC) $(CFLAGS) -Wa,-adhln=$*.lst $< -o $@

psc3_erase.o: psc3_erase.c
	$(CC) $(CFLAGS) -Wa,-adhln=$*.lst $< -o $@

%.bin: %.o
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin Makefile
	$(BIN2C) < $< > $@

clean:
	-rm -f *.o *.lst *.bin *.inc
