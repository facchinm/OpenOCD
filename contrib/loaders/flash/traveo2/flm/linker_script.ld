EXTERN(Init)
EXTERN(UnInit)
EXTERN(EraseSector)
EXTERN(ProgramPage)
EXTERN(BlankCheck)
EXTERN(EraseChip)
EXTERN(Verify)
EXTERN(FlashDevice)
EXTERN(SEGGER_FL_Read)

PHDRS
{
  PrgAndData PT_LOAD;
  DevDscr PT_LOAD;
}

SECTIONS
{
	PrgCode :
	{
		*(.text .text.*)
	} :PrgAndData

	PrgData :
	{
		LONG(0xA5A5A5A5) /* Force this section to have PT_LOAD attribute */
		*(.rodata EXCLUDE_FILE(*FlashDev.o) .rodata*)
		*(.rdata)
		*(.data .data.*)
		*(.bss .bss.*)
		LONG(0x5A5A5A5A) /* Force this section to have PT_LOAD attribute */
	} :PrgAndData

	DevDscr :
	{
		*(.rodata.FlashDevice)
	} :DevDscr

	/DISCARD/ :
	{
		*(.got .got*)
		*(.plt .plt*)
		*(COMMON)
		*(.init)
		*(.fini)
		*(.ARM.attributes)
		*(.comment)
	}
}
