OUTPUT_ARCH(mips)
ENTRY(start)
MEMORY {
 /* libdragon's header's entry point is 80000400. */
 ram : ORIGIN = 0xFFFFffff80000400, LENGTH = 8M - 0x400
 /* N64 program starts from 0x1000. */
 rom : ORIGIN = 0x10001000, LENGTH = 32M - 0x1000
}
SECTIONS
{
 /* .text contains loader */
 .text : {
  *(.text .text.*)
 } > ram AT> rom
 /* .data contains vmlinux.bin */
 .data ALIGN(8) : { /* 8-bytes align is PI DMA constraint */
  *(.data .data.*)
 } > ram AT> rom
 vmlinux_bin_start_load = LOADADDR(.data);
 /* Sections to be discarded */
 /DISCARD/ : { *(.exit.text) *(.memexit.text) *(.exit.data) *(.memexit.data) *(.memexit.rodata) *(.exitcall.exit) *(.discard) *(.discard.*) }
 /DISCARD/ : {
  /* ABI crap starts here */
  *(.MIPS.abiflags)
  *(.MIPS.options)
  *(.options)
  *(.pdr)
  *(.reginfo)
  *(.eh_frame)
 }
}
