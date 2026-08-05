/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeMX
**
**  Abstract    : Linker script for the MCUboot-managed STM32H743 application.
**                The first 1 KiB of the primary slot is the MCUboot header;
**                the end of the slot is reserved for TLVs and swap trailer.
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
**   1. Redistributions of source code must retain the above copyright notice,
**      this list of conditions and the following disclaimer.
**   2. Redistributions in binary form must reproduce the above copyright notice,
**      this list of conditions and the following disclaimer in the documentation
**      and/or other materials provided with the distribution.
**   3. Neither the name of STMicroelectronics nor the names of its contributors
**      may be used to endorse or promote products derived from this software
**      without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x0;        /* CubeMX/newlib heap disabled; platform heap is in RAM_D1 */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
RAM_D1 (xrw)       : ORIGIN = 0x24000000, LENGTH = 512K
RAM_D2 (xrw)       : ORIGIN = 0x30000000, LENGTH = 256K
RAM_DMA (xrw)      : ORIGIN = 0x30040000, LENGTH = 32K
RAM_D3 (xrw)       : ORIGIN = 0x38000000, LENGTH = 64K
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
FLASH (rx)         : ORIGIN = 0x08040400, LENGTH = 0x000BF000
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* Flash programming code runs from DTCM. It is copied together with .data
     by the existing startup loop, but retains executable-only permissions. */
  .dima_ramfunc :
  {
    . = ALIGN(4);
    _sdata = .;
    KEEP(*(.dima_ramfunc))
    KEEP(*(.dima_ramfunc*))
    . = ALIGN(32);
  } >DTCMRAM AT> FLASH

  /* used by the startup to initialize DTCM code and data */
  _sidata = LOADADDR(.dima_ramfunc);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    __dima_orb_meta_start__ = .;
    KEEP(*(.dima_orb_meta))
    __dima_orb_meta_end__ = .;

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >DTCMRAM AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at data start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >DTCMRAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM

  /* DMA buffers are isolated in D2 SRAM and aligned to cache lines. */
  .dima_dma ORIGIN(RAM_DMA) (NOLOAD) :
  {
    . = ALIGN(32);
    __dima_dma_region_start__ = .;
    __dima_dma_start__ = .;
    KEEP(*(.dima_dma))
    KEEP(*(.dima_dma*))
    __dima_dma_end__ = .;
    . = ALIGN(32);
  } >RAM_DMA
  __dima_dma_region_end__ = ORIGIN(RAM_DMA) + LENGTH(RAM_DMA);

  /* Dima general heap lives in D1 AXI SRAM; DTCM and DMA-oriented D2 stay isolated. */
  .dima_heap (NOLOAD) :
  {
    . = ALIGN(32);
    __dima_heap_start__ = .;
    . = . + 256K;
    __dima_heap_end__ = .;
    . = ALIGN(32);
  } >RAM_D1

  /* FreeRTOS task stacks come from a fixed platform pool outside the heap. */
  .dima_task_pool (NOLOAD) :
  {
    . = ALIGN(32);
    __dima_task_pool_start__ = .;
    KEEP(*(.dima_task_pool))
    KEEP(*(.dima_task_pool*))
    __dima_task_pool_end__ = .;
    . = ALIGN(32);
  } >RAM_D1

  /* SRAM capture is persisted to 0x08020000 for ROM USB DFU readback. */
  .dima_boot_diag (NOLOAD) :
  {
    . = ALIGN(32);
    __dima_boot_diag_start__ = .;
    KEEP(*(.dima_boot_diag))
    __dima_boot_diag_end__ = .;
    . = ALIGN(32);
  } >RAM_D3

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }

  ASSERT(ADDR(.isr_vector) == 0x08040400,
         "Application vector table must follow the 0x400-byte MCUboot header")
  ASSERT(ADDR(.dima_boot_diag) == 0x38000000,
         "Boot diagnostics must stay at the documented fixed address")
  ASSERT(SIZEOF(.dima_boot_diag) <= 0x200,
         "Boot diagnostics record exceeds its fixed 512-byte budget")
  ASSERT((ADDR(.data) - ADDR(.dima_ramfunc)) ==
         (LOADADDR(.data) - LOADADDR(.dima_ramfunc)),
         "Startup DTCM copy span must have identical load and run gaps")
  ASSERT(ADDR(.dima_dma) == 0x30040000,
         "DMA region must start at D2 SRAM3 base")
  ASSERT((ADDR(.dima_dma) & 31) == 0,
         "DMA region must be cache-line aligned")
  ASSERT(SIZEOF(.dima_dma) <= 32K,
         "DMA buffers exceed the fixed 32 KiB SRAM3 region")
  ASSERT(ADDR(.dima_task_pool) >= 0x24040000,
         "Task pool must not overlap the 256 KiB D1 heap")
  ASSERT(SIZEOF(.dima_task_pool) <= 48K,
         "Task pool exceeds its fixed platform budget")
}
