ifeq (x$(TARGET),x)
$(error TARGET variable not defined!)
endif

CPU_FLAGS   := -mthumb -mcpu=cortex-m0 -msoft-float
include $(TARGET)/options.mk
	
AS_SRC    :=

OUT_DIR   :=  out
VERBOSE_BUILD ?= 0

CC       := arm-none-eabi-gcc
AS       := arm-none-eabi-gcc -x assembler-with-cpp
LD       := arm-none-eabi-ld
SIZE     := arm-none-eabi-size
READELF  := arm-none-eabi-readelf
OBJCOPY  := arm-none-eabi-objcopy
OBJDUMP  := arm-none-eabi-objdump
STRIP    := arm-none-eabi-strip

CC_OBJ   := $(CC_SRC:.c=.o)
AS_OBJ   := $(AS_SRC:.s=.o)
OBJ      := $(CC_OBJ) $(AS_OBJ)
DEPFILES := $(OBJ:.o=.d)
OUT_DIR    ?= out/

##############################################################################################################################
# Debug/Release options
##############################################################################################################################
OPTFLAGS := -Os
GLOBAL_DEFS += -DNDEBUG -DLOADER_TARGET=\"$(TARGET)\"

##############################################################################################################################
# Warning options
##############################################################################################################################
CC_WARN := -Wall -Wextra -Wno-unused-variable -Wno-unused-function -Wno-unused-but-set-variable -Wno-return-type -Wno-maybe-uninitialized \
			-Wno-comment -Wno-unused-parameter -Wno-expansion-to-defined -Wno-sign-compare -Wno-absolute-value -Wno-cpp

##############################################################################################################################
# Code generation options
##############################################################################################################################
CC_CODEGEN := -ffunction-sections -fdata-sections -fsingle-precision-constant -ffast-math -fmerge-all-constants \
			   -fstrict-overflow -fno-math-errno -frename-registers -ffreestanding

##############################################################################################################################
# Complete command lines
##############################################################################################################################
CFLAGS      := -std=c11   $(CC_WARN)  $(CC_CODEGEN)  $(CPU_FLAGS) $(OPTFLAGS) $(INCLPATHS) $(GLOBAL_DEFS) \
			-fpic -mapcs-frame -fwhole-program -static
			
LDFLAGS     := $(CPU_FLAGS) $(OPTFLAGS) $(GLOBAL_DEFS) -T$(LDSCRIPT) -Wl,--gc-sections,--nmagic,-static,--no-warn-rwx-segments \
			--specs=nano.specs --specs=nosys.specs

##############################################################################################################################
# Build recipes
##############################################################################################################################
.PHONY: all clean

all: $(OUT_DIR)/$(TARGET).elf
	@$(SIZE) $(OUT_DIR)/$(TARGET).elf | tail -n 1 | awk '{printf "\nFlash: %8.8s\nRAM:   %8.8s\n\n", $$1+$$2, $$2+$$3}'
	@$(OBJDUMP) -DlS $(OUT_DIR)/$(TARGET).elf > $(OUT_DIR)/$(TARGET).s
	@mv $(OUT_DIR)/$(TARGET).elf ../../../../flm/infineon/traveo2/
	
	@touch $(TARGET).creator
	
	@echo "/* Auto-generated with a makefile */\n" > $(TARGET).config;
	@for DEFINE in $(GLOBAL_DEFS); do \
		echo $$DEFINE | sed "s/-D/#define /;s/=/ /" >> $(TARGET).config; \
	done
	@echo "/* Auto-generated with a makefile */\n" > $(TARGET).includes
	@for INCLUDE in $(INCLPATHS); do \
		echo $$INCLUDE | sed "s/-I//g" >> $(TARGET).includes; \
	done
	@cat $(DEPFILES) | grep -oP '^ (.*)' | sed 's/ /\n/g;s/\\//g' | sort -u > $(TARGET).files
	@find $(TARGET)/ -regex '.*\.\(mk\|ld\|c\|h\)' >> $(TARGET).files
	@echo "Makefile" >> $(TARGET).files
	@echo "README.md" >> $(TARGET).files
	@find -L  -type f -name "*.[od]" -delete

clean:
	@echo Cleaning up...
	@find -L  -type f -name "*.[od]" -delete
	@rm -rf $(OUT_DIR)/*
	@echo > $(TARGET).files
	@echo > $(TARGET).includes
	@echo > $(TARGET).config

$(OUT_DIR)/$(TARGET).elf:$(OBJ)
	@echo [$(TARGET): ld ] $(TARGET).elf
	@mkdir -p $(OUT_DIR)
	@$(CC) $(LDFLAGS) $(OBJ) -o $(OUT_DIR)/$(TARGET).elf
	@$(STRIP) --strip-debug $(OUT_DIR)/$(TARGET).elf

%.o:%.c
	@echo [$(TARGET): gcc] `basename $*.c`
	@$(CC) $(CFLAGS) -MMD -c $*.c -o $*.o

-include $(DEPFILES)
