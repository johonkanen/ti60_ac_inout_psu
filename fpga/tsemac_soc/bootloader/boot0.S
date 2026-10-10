/*
 * First stage boot code in the SoC's 2 KB on-chip RAM, at the reset address 0xf9000000:
 * a stack at the top of that RAM, then boot0.c.
 */
	.section .start, "ax"
	.globl _start
_start:
	csrw mie, zero
	csrci mstatus, 8
	lui sp, %hi(0xf9000800)
	addi sp, sp, %lo(0xf9000800)
	call main
1:	wfi
	j 1b
