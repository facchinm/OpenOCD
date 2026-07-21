GLOBAL_DEFS += -DCY_USE_PSVP=0 -Dtviic2d6mddr -DCYT4ENDBAS=1 -DCY_MCU_rev_a=1 -DUSED_SMIF_CHANNEL=0 -DCY_500BGA_EVK_rev_a=1 \
			   -DSECTOR_ARCH=SECTORS_UNIFORM -DROW_SIZE=512

#GLOBAL_DEFS += -DSECTOR_ARCH=SECTORS_UNIFORM
#GLOBAL_DEFS += -DSECTOR_ARCH=SECTORS_HYBRID_B
#GLOBAL_DEFS += -DSECTOR_ARCH=SECTORS_HYBRID_T
#GLOBAL_DEFS += -DSECTOR_ARCH=SECTORS_HYBRID_BT

LDSCRIPT := flm/linker_script.ld

CC_SRC += \
	$(TARGET)/FlashPrg.c \
	$(TARGET)/FlashDev.c

INCLPATHS :=											\
  -Iflm													\
  -Isdl/common											\
  -Isdl/common/hdr										\
  -Isdl/common/src										\
  -Isdl/common/src/drivers								\
  -Isdl/common/src/mw									\
  -Isdl/tviic2d6mddr									\
  -Isdl/tviic2d6mddr/hdr								\
  -Isdl/tviic2d6mddr/hdr/rev_a							\
  -Isdl/tviic2d6mddr/hdr/rev_a/ip						\
  -Isdl/tviic2d6mddr/hdr/rev_a/mcureg					\
  -Isdl/tviic2d6mddr/src								\
  -Isdl/tviic2d6mddr/src/drivers						\
  -Isdl/tviic2d6mddr/src/examples/smif/S28H/			\
  -Isdl/tviic2d6mddr/src/interrupts						\
  -Isdl/tviic2d6mddr/src/interrupts/rev_a				\
  -Isdl/tviic2d6mddr/src/mw								\
  -Isdl/tviic2d6mddr/src/system							\
  -Isdl/tviic2d6mddr/src/system/rev_a					\
  -Isdl/common/hdr/cmsis/include/
