/*
 * Both harts arrive here at 0x01000000, where the design's bootloader copied the loader
 * and where the image will go. Hart 0 copies the loader to its link address and runs
 * main() there. Hart 1 waits in the original copy until hart 0 says the move is done,
 * then parks in the moved copy until main() hands it the image's entry.
 *
 * A warm reboot comes through the bootloader with hart 1 still parked in Zephyr, which
 * never releases it: hart 0 then goes on alone (main.c waits only a little for it).
 */
#include "../module/include/sapphire_loader.h"

#define ABS(reg, sym) lui reg, %hi(sym); addi reg, reg, %lo(sym)
#define DCACHE_INVALIDATE .word 0x500f

	.section .start, "ax"
	.globl _start
_start:
	.option push
	.option norvc /* a 4-byte jump, so the magic is the second word */
	j entry
	.option pop
	.word LOADER_MAGIC
	.word _image_end - _start /* bytes the bootloader must have copied */

entry:
	csrw mie, zero
	csrci mstatus, 8
	lla a0, _start /* where this copy runs */
	csrr t0, mhartid
	bnez t0, secondary

	/* hart 0: move the loader to its link address */
	ABS(a1, _start)
	ABS(a2, _image_end)
	mv a3, a0
copy:
	lw t1, 0(a3)
	sw t1, 0(a1)
	addi a3, a3, 4
	addi a1, a1, 4
	bltu a1, a2, copy

	/* clear bss there, which holds the flags hart 1 waits on in the moved copy */
	ABS(a1, _bss_start)
	ABS(a2, _bss_end)
clear:
	bgeu a1, a2, cleared
	sw zero, 0(a1)
	addi a1, a1, 4
	j clear
cleared:
	fence
	fence.i

	/* release hart 1 from the original copy: the flag there, at its run address */
	lla t1, moved
	li t2, 1
	sw t2, 0(t1)
	fence

	ABS(sp, _stack_top)
	ABS(t0, main)
	jr t0

secondary:
	/* hart 1, in the original copy: wait for hart 0 to have moved the loader */
	lla t1, moved
1:	DCACHE_INVALIDATE
	lw t2, 0(t1)
	beqz t2, 1b
	ABS(t0, park)
	jr t0

park:
	/* in the moved copy: say so, then wait for the image's entry */
	ABS(t1, secondary_parked)
	li t2, 1
	sw t2, 0(t1)
	fence
	ABS(t1, secondary_entry)
2:	DCACHE_INVALIDATE
	lw t2, 0(t1)
	beqz t2, 2b
	fence.i
	jr t2

	/* in .start, not .bss: it must read 0 in the copy the bootloader just made */
	.balign 4
moved:
	.word 0
