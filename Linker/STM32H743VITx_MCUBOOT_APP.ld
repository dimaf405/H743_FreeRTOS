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
**                Primary slot 基址为 0x08040000：0x400 字节 MCUboot header
**                之后才是向量表；尾部 0xC00 为 TLV/swap trailer 预留，
**                因而应用 FLASH 可用长度固定为 0xBF000。
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
/* 启动栈位于 DTCM 顶端；newlib/CubeMX heap 禁用，通用平台堆另放 RAM_D1。 */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x0;        /* CubeMX/newlib heap disabled; platform heap is in RAM_D1 */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Specify the memory areas */
/* RAM_DMA 单独占 D2 SRAM3，避免 D-cache 一致性风险；RAM_D3 保存复位后诊断。 */
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

  /* Flash programming code runs from DTCM while ordinary initialized data
     lives in D2 SRAM1/2. */
  /* 擦写 Bank1 时 Flash 不可取指，因此 ramfunc 必须保留在 DTCM；它与位于
     D2 的 .data 地址不连续，Reset_Handler 必须分别完成两段复制。 */
  .dima_ramfunc :
  {
    . = ALIGN(4);
    _sramfunc = .;
    KEEP(*(.dima_ramfunc))
    KEEP(*(.dima_ramfunc*))
    . = ALIGN(32);
    _eramfunc = .;
  } >DTCMRAM AT> FLASH
  _siramfunc = LOADADDR(.dima_ramfunc);

  /* 普通初始化数据固定从 D2 SRAM1 基址开始，加载副本仍顺序保存在 Flash。 */
  .data ORIGIN(RAM_D2) :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM_D2 AT> FLASH
  _sidata = LOADADDR(.data);

  /* 普通零初始化数据紧随 .data 放入 D2 SRAM1/2；DMA 专用 SRAM3 不参与。 */
  .bss (NOLOAD) :
  {
    . = ALIGN(32);
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at data start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(32);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM_D2

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
  /* NOLOAD 避免镜像携带无意义初值；起止均按 32 字节 cache line 对齐。 */
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
  /* 固定 256 KiB 平台堆与随后 task pool 分区，链接 ASSERT 防止二者重叠。 */
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
  /* 默认 .data/.bss 必须完整位于 D2 SRAM1/2，防止后续链接规则静默退回 DTCM。 */
  ASSERT(ADDR(.data) == ORIGIN(RAM_D2),
         "Application .data must start at D2 SRAM1 base")
  ASSERT(ADDR(.bss) >= ADDR(.data) + SIZEOF(.data),
         "Application .bss must follow .data in D2 SRAM")
  ASSERT(ADDR(.bss) + SIZEOF(.bss) <= ORIGIN(RAM_D2) + LENGTH(RAM_D2),
         "Application .data/.bss exceed D2 SRAM1/2 capacity")
  /* DTCM 低 64 KiB 是静态占用硬上限，至少为向下增长的 MSP 留出 64 KiB。 */
  ASSERT(ADDR(._user_heap_stack) + SIZEOF(._user_heap_stack) <=
         ORIGIN(DTCMRAM) + 64K,
         "DTCM static footprint exceeds the 64 KiB budget")
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
