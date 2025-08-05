OUTPUT_ARCH( "riscv" )
ENTRY(_start)

_text_start = 0xaaaaa000;
_stack_top  = 0xf0000000;

SECTIONS
{

  . = _text_start;

  .init :
  {
    . = ALIGN(4096);
    *(.init)
    . = ALIGN(4096);
  }

  . = .;

  .text :
  {
    . = ALIGN(4096);
    _text_vma_start = .;

    *(.text)
    *(.text*)

    _text_vma_end = .;
    . = ALIGN(4096);
  }

  . = .;

  .data :
  {
    . = ALIGN(4096);
    _data_vma_start = .;

    *(.*data*)

    _data_vma_end = .;
    . = ALIGN(4096);
  }

  . = .;

  .bss :
  {
    . = ALIGN(4096);
    _bss_vma_start = .;

    *(.*bss*)

    _bss_vma_end = .;
    . = ALIGN(4096);
  }

  __global_pointer$ = .;

  _end = .;

  /DISCARD/ :
  {
    *(.eh_frame)
    *(.riscv.attributes)
    *(.comment)
    *(.debug*)
  }
}
