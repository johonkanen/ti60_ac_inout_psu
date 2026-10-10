/*
 * The design's bootloader copies the loader to 0x01000000, where the image goes, so the
 * loader moves itself to the top of HyperRAM first (start.S). Zephyr's RAM takes this
 * space back once it runs.
 */
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
	RAM (rwx) : ORIGIN = 0x01ff0000, LENGTH = 64K
}

SECTIONS
{
	.text : {
		KEEP(*(.start))
		*(.text .text.*)
		*(.rodata .rodata.* .srodata .srodata.*)
	} > RAM
	.data : {
		*(.data .data.* .sdata .sdata.*)
		. = ALIGN(4);
	} > RAM
	_image_end = .;
	.bss (NOLOAD) : {
		_bss_start = .;
		*(.bss .bss.* .sbss .sbss.* COMMON)
		. = ALIGN(4);
		_bss_end = .;
	} > RAM
	.stack (NOLOAD) : {
		. = ALIGN(16);
		. += 4096;
		_stack_top = .;
	} > RAM
	/DISCARD/ : { *(.comment) *(.riscv.attributes) *(.note*) *(.eh_frame*) }
}
