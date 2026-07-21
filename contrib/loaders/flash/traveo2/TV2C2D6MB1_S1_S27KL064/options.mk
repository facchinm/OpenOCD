GLOBAL_DEFS += -DCY_USE_PSVP=0 -Dtviic2d6m -DCYT4DNJBRS=1 -DCY_MCU_rev_c=1 -DUSED_SMIF_CHANNEL=1 -DROW_SIZE=512

LDSCRIPT := flm/linker_script_initonly.ld

CC_SRC += \
	$(TARGET)/FlashPrg.c

INCLPATHS :=											\
  -Iflm													\
  -Isdl/common											\
  -Isdl/common/hdr										\
  -Isdl/common/hdr/cmsis								\
  -Isdl/common/hdr/cmsis/include						\
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
