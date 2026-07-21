GLOBAL_DEFS += -DCY_USE_PSVP=0 -Dtviic2d6m -DCYT4DNJBRS=1 -DCY_MCU_rev_c=1 -DUSED_SMIF_CHANNEL=1 \
			   -DSECTOR_ARCH=SECTORS_UNIFORM

GLOBAL_DEFS += -DSECTOR_ARCH=SECTORS_UNIFORM
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
  -Isdl/tviic2d6m										\
  -Isdl/tviic2d6m/hdr									\
  -Isdl/tviic2d6m/hdr/rev_c								\
  -Isdl/tviic2d6m/hdr/rev_c/ip							\
  -Isdl/tviic2d6m/hdr/rev_c/mcureg						\
  -Isdl/tviic2d6m/src									\
  -Isdl/tviic2d6m/src/drivers							\
  -Isdl/tviic2d6m/src/examples/smif						\
  -Isdl/tviic2d6m/src/interrupts						\
  -Isdl/tviic2d6m/src/interrupts/rev_c					\
  -Isdl/tviic2d6m/src/mw								\
  -Isdl/tviic2d6m/src/system							\
  -Isdl/tviic2d6m/src/system/rev_c						\
  -Isdl/common/hdr/cmsis/include/
