GLOBAL_DEFS += -DCY_USE_PSVP=0 -Dtviice4m -DCYT2CL8BAS=1 -DCY_MCU_rev_a=1 -DCY_176LQFP_EVK_REV_A=1 -DROW_SIZE=512

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
  -Isdl/tviice4m										\
  -Isdl/tviice4m/hdr									\
  -Isdl/tviice4m/hdr/rev_a								\
  -Isdl/tviice4m/hdr/rev_a/ip							\
  -Isdl/tviice4m/hdr/rev_a/mcureg						\
  -Isdl/tviice4m/src									\
  -Isdl/tviice4m/src/drivers							\
  -Isdl/tviice4m/src/examples/smif						\
  -Isdl/tviice4m/src/interrupts							\
  -Isdl/tviice4m/src/interrupts/rev_a					\
  -Isdl/tviice4m/src/mw									\
  -Isdl/tviice4m/src/system								\
  -Isdl/tviice4m/src/system/rev_a						\
