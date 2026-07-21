BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS =  -c -mthumb -mcpu=cortex-m0plus -Os
CFLAGS += -Wall -Wextra

all: psoc6_write.inc psoc62m_write.inc tv2_write.inc psoc6_erase.inc psoc62m_erase.inc tv2_erase.inc

.PHONY: clean

.INTERMEDIATE: psoc6_write.o psoc62m_write.o tv2_write.o psoc6_erase.o psoc62m_erase.o tv2_erase.o

psoc6_write.o: psoc6_write.c device.h flashboot.h
	$(CC) $(CFLAGS) -DMXS40_VARIANT=MXS40_VARIANT_PSOC6A_BLE -Wa,-adhln=$*.lst $< -o $@

psoc62m_write.o: psoc6_write.c device.h flashboot.h
	$(CC) $(CFLAGS) -DMXS40_VARIANT=MXS40_VARIANT_PSOC6A_2M -Wa,-adhln=$*.lst $< -o $@

tv2_write.o: psoc6_write.c device.h flashboot.h
	$(CC) $(CFLAGS) -DMXS40_VARIANT=MXS40_VARIANT_TRAVEO_II -Wa,-adhln=$*.lst $< -o $@

psoc6_erase.o: psoc6_erase.c device.h flashboot.h
	$(CC) $(CFLAGS) -DMXS40_VARIANT=MXS40_VARIANT_PSOC6A_BLE -Wa,-adhln=$*.lst $< -o $@

psoc62m_erase.o: psoc6_erase.c device.h flashboot.h
	$(CC) $(CFLAGS) -DMXS40_VARIANT=MXS40_VARIANT_PSOC6A_2M -Wa,-adhln=$*.lst $< -o $@

tv2_erase.o: psoc6_erase.c device.h flashboot.h
	$(CC) $(CFLAGS) -DMXS40_VARIANT=MXS40_VARIANT_TRAVEO_II -Wa,-adhln=$*.lst $< -o $@

%.bin: %.o
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.o *.lst *.bin *.inc
