OUTPUT_ARCH(riscv)
ENTRY(_start)
MEMORY { OCR (rwx) : ORIGIN = 0xf9000000, LENGTH = 2K }
SECTIONS
{
	.text : { KEEP(*(.start)) *(.text .text.*) *(.rodata .rodata.* .srodata .srodata.*) } > OCR
	.data : { *(.data .data.* .sdata .sdata.*) *(.bss .bss.* .sbss .sbss.* COMMON) } > OCR
	/DISCARD/ : { *(.comment) *(.riscv.attributes) *(.note*) *(.eh_frame*) }
}
