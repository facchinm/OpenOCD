ENTRY(Init)

PHDRS
{
  PrgAndData PT_LOAD;
}

SECTIONS
{
	PrgCode 0x0000 :
	{
		*(.start*)
		*(.text .text.*)
	} :PrgAndData

	PrgData 0x2000 :
	{
		*(.rodata EXCLUDE_FILE(*FlashDev.o) .rodata*)
		*(.data .data.*)
		*(.bss .bss.*)
		*(.got .got*)
	} :PrgAndData

	/DISCARD/ :
	{
		*(.rodata.FlashDevice)
		*(.init)
		*(.fini)
		*(.ARM.attributes)
		*(.comment)
		*(COMMON)
	}
}
