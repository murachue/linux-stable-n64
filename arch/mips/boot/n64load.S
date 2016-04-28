	.set reorder
	.set at

	.text

.type start, %function
.globl start
start:
	/* init stack to memsize-0x10, at kseg0 */
	lw $t0, 0x80000318 /* memsize, if CIC_6105 this is 0x800003F0? */
	li $t1, 0x80000000 - 0x10
	addu $sp, $t0, $t1

	/* magic n64 hardware init */
	li $t0, 8
	sw $t0, 0xBFC007FC

	/* reloc this code */
	li $t0, 0x803FF000 /* near 4MB */
	la $t1, start
	la $t2, end
1:
	lw $t3, ($t1)
	addiu $t1, 4
	sw $t3, ($t0)
	addiu $t0, 4
	blt $t1, $t2, 1b

	/* jump to relocated code */
	/* j 0x003FF000 + 1f - start;; Error: unsupported constant in relocation */
	/* li $t0, 0x803FF000 + (1f - start);; this goes wrong... why? */
	/* la $t0, 0x803FF000 + (1f - start);; Error: expression too complex */
	li $t0, 0x803FF000
	addiu $t0, 1f - start
	jr $t0
1:

	/* flush I/D cache */
	/* do this before DMA from cart, because could not just invalidate but write-back-invalidate Dcache. */
	/* TODO research cache line size (assuming I=32B, D=16B), or dynamically fetch from CP0 reg */
	li $t0, 0
	li $t1, 0
	li $t2, 16*1024 /* 16K Icache 8K Dcache */
1:
	cache 0, 0x00($t0) /* I index inval */
	cache 1, 0x00($t1) /* D wback inval */
	addiu $t0, 32
	addiu $t1, 16
	blt $t0, $t2, 1b

	/* load vmlinux */
	/* we don't wait DMA, we assume DMA is ready. */
	la $t1, 0xA4600000 /* PI base */
	sw $zero, 0x00($t1) /* PI_DRAM_ADDR_REG, phys-ram addr = 0 */
	la $t0, vmlinux_bin_start_load /* it is phys-addr calc'd at ldscript */
	sw $t0, 0x04($t1) /* PI_CART_ADDR_REG */
	la $t0, vmlinux_bin_size - 1 /* note: Length must be >0. At most 7 bytes are xferred more. */
	sw $t0, 0x0C($t1) /* PI_WR_LEN_REG, start DMA cart->dram */
1:
	lw $t0, 0x10($t1) /* PI_STATUS_REG */
	andi $t0, 1 /* DMA busy */
	bnez $t0, 1b

	/* jump to kernel_start */
	j 0x80000400
end:

	.data
vmlinux_bin_start:
	.incbin VMLINUX_BIN
	.set vmlinux_bin_size, (. - vmlinux_bin_start)
