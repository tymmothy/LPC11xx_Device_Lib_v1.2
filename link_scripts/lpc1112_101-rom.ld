/******************************************************************************/
/*    Output Options, Entry Symbol, Libraries to Aggressively Use             */
/******************************************************************************/

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
STARTUP(lpc11xx_crt0.o)
ENTRY(_start)
GROUP(-lgcc -lc -llpc11xx)

/******************************************************************************/
/*                             Tweakable Stuff                                */
/******************************************************************************/

__Minimum_Stack_Size__ = 0x100 ;

/******************************************************************************/
/*                           Exception Handlers                               */
/******************************************************************************/

/* By default all un-set IRQ handlers go to reset */

PROVIDE ( Reset_Handler = _start ) ;

PROVIDE ( UndefinedException_Handler = Reset_Handler ) ;

PROVIDE ( HardFault_Handler = UndefinedException_Handler ) ;
PROVIDE ( SVC_Handler = UndefinedException_Handler ) ;
PROVIDE ( PendSV_Handler = UndefinedException_Handler ) ;
PROVIDE ( SysTick_Handler = UndefinedException_Handler ) ;

PROVIDE ( WAKEUP0_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP1_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP2_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP3_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP4_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP5_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP6_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP7_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP8_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP9_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP10_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP11_IRQHandler = Reset_Handler ) ;
PROVIDE ( WAKEUP12_IRQHandler = Reset_Handler ) ;
PROVIDE ( SSP1_IRQHandler = Reset_Handler ) ;
PROVIDE ( I2C0_IRQHandler = Reset_Handler ) ;
PROVIDE ( CT16B0_IRQHandler = Reset_Handler ) ;
PROVIDE ( CT16B1_IRQHandler = Reset_Handler ) ;
PROVIDE ( CT32B0_IRQHandler = Reset_Handler ) ;
PROVIDE ( CT32B1_IRQHandler = Reset_Handler ) ;
PROVIDE ( SSP0_IRQHandler = Reset_Handler ) ;
PROVIDE ( UART0_IRQHandler = Reset_Handler ) ;
PROVIDE ( ADC0_IRQHandler = Reset_Handler ) ;
PROVIDE ( WDT_IRQHandler = Reset_Handler ) ;
PROVIDE ( BOD_IRQHandler = Reset_Handler ) ;
PROVIDE ( GPIO3_IRQHandler = Reset_Handler ) ;
PROVIDE ( GPIO2_IRQHandler = Reset_Handler ) ;
PROVIDE ( GPIO1_IRQHandler = Reset_Handler ) ;
PROVIDE ( GPIO0_IRQHandler = Reset_Handler ) ;

/******************************************************************************/
/*                              Peripheral Map                                */
/******************************************************************************/

/* Peripheral base address */
PERIPH_BASE           = 0x40000000;

/* Peripheral memory map */
APBPERIPH_BASE       = PERIPH_BASE ;
AHBPERIPH_BASE       = (PERIPH_BASE + 0x10000000) ;

PROVIDE ( I2C0       = (APBPERIPH_BASE + 0x00000) ) ;
PROVIDE ( WDT        = (APBPERIPH_BASE + 0x04000) ) ;
PROVIDE ( UART0      = (APBPERIPH_BASE + 0x08000) ) ;
PROVIDE ( CT16B0     = (APBPERIPH_BASE + 0x0C000) ) ;
PROVIDE ( CT16B1     = (APBPERIPH_BASE + 0x10000) ) ;
PROVIDE ( CT32B0     = (APBPERIPH_BASE + 0x14000) ) ;
PROVIDE ( CT32B1     = (APBPERIPH_BASE + 0x18000) ) ;
PROVIDE ( ADC0       = (APBPERIPH_BASE + 0x1C000) ) ;
PROVIDE ( PMU        = (APBPERIPH_BASE + 0x38000) ) ;
PROVIDE ( FLASH      = (APBPERIPH_BASE + 0x3C000) ) ;
PROVIDE ( SPI0       = (APBPERIPH_BASE + 0x40000) ) ;
PROVIDE ( IOCON      = (APBPERIPH_BASE + 0x44000) ) ;
PROVIDE ( SYSCON     = (APBPERIPH_BASE + 0x48000) ) ;

PROVIDE ( GPIO0      = (AHBPERIPH_BASE + 0x0000) ) ;
PROVIDE ( GPIO1      = (AHBPERIPH_BASE + 0x1000) ) ;
PROVIDE ( GPIO2      = (AHBPERIPH_BASE + 0x2000) ) ;
PROVIDE ( GPIO3      = (AHBPERIPH_BASE + 0x3000) ) ;

/* System Control Space memory map */
SCS_BASE              = 0xE000E000;

PROVIDE ( SysTick         = (SCS_BASE + 0x0010) ) ;
PROVIDE ( NVIC            = (SCS_BASE + 0x0100) ) ;
PROVIDE ( SYSCONTROL      = (SCS_BASE + 0x0D00) ) ;

/******************************************************************************/
/*                                 Memory Map                                 */
/******************************************************************************/

MEMORY {
  flash (rx) : ORIGIN = 0x00000000, LENGTH = 16K
  ram (xrw)  : ORIGIN = 0x10000000, LENGTH = 2K
}

/******************************************************************************/
/*                              Section Layout                                */
/******************************************************************************/

/* Leave RAM space for IAP Commands */
_stack = ORIGIN(ram) + LENGTH(ram) - 36;
_stack_size = __Minimum_Stack_Size__;

__checksum__ = ~(_stack + Reset_Handler + HardFault_Handler) - 1;

SECTIONS {
    .text : {
        . = ALIGN(4);
        KEEP(*(.boot_isr_vector))

        . = 0x2fc;
        KEEP(*(.crp))

        . = 0x300;
        KEEP(*(.startup .startup.*))
        CREATE_OBJECT_SYMBOLS
        . = ALIGN(4);
        *(.text .text.* .gnu.linkonce.t.*)
        *(.plt)
        *(.gnu_warning)
        *(.glue_7t) *(.glue_7)
                
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))
         
        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))
         
        . = ALIGN(4); 
        *(.rodata .rodata.* .gnu.linkonce.r.*)
         
        . = ALIGN(4);
        KEEP(*(.init0))
        KEEP(*(.init1))
        KEEP(*(.init2))
        KEEP(*(.init3))
        KEEP(*(.init4))
        KEEP(*(.init5))
        KEEP(*(.init6))
        KEEP(*(.init7))
        KEEP(*(.init8))
        KEEP(*(.init9))
        
        . = ALIGN(4);
        _init_array_start = ABSOLUTE(.);
        KEEP(*(.init_array))
        _init_array_end = ABSOLUTE(.);
                 
        . = ALIGN(4);
        _fini_array_start = ABSOLUTE(.);
        KEEP(*(.fini_array))
        _fini_array_end = ABSOLUTE(.);
        
        . = ALIGN(4);
        KEEP(*(.fini9))
        KEEP(*(.fini8))
        KEEP(*(.fini7))
        KEEP(*(.fini6))
        KEEP(*(.fini5))
        KEEP(*(.fini4))
        KEEP(*(.fini3))
        KEEP(*(.fini2))
        KEEP(*(.fini1))
        KEEP(*(.fini0))
        
        . = ALIGN(4);
    } > flash

    .ARM.exidx : { 
        *(.ARM.exidx* .gnu.linkonce.armexidx.*) 
       } >flash

    /* C++ exception unwinding stuff--needed by Code Sourcery toolchain */
    .ARM.extab : { 
        *(.ARM.extab* .gnu.linkonce.armextab.*) 
       } >flash
       
    __end_of_text__ = .;

    .data : {
        . = ALIGN(4);
        _data_src_start = LOADADDR(.data);
        _data_start = .;
        KEEP(*(.ram_isr_vector))
        KEEP(*(.jcr))
        *(.got.plt) *(.got)
        *(.fastcode)
        *(.shdata)
        *(.data .data.* .gnu.linkonce.d.*)
        . = ALIGN(4);
    } > ram AT > flash    

    _data_end = .;
    
    .dataflash : {
        . = ALIGN(4096);
        KEEP(*(.dataflash))
        . = ALIGN(4096);
    } > flash

    .bss : {
        . = ALIGN(4);
        _bss_start = ABSOLUTE(.);
        *(.shbss)
        *(.bss .bss.* .gnu.linkonce.b.*)
        . = ALIGN(4);
        . += 4;
    } > ram

    _bss_end = .;
    
    _end = .;
    PROVIDE (end = .);
    _heap_start = .;
    _heap_size  = _stack - __Minimum_Stack_Size__ - _heap_start;
    
    ._checkstack : {
        /* User stack section; meant to guarantee enough space left for it */
        . = ALIGN(4);
        __checkstack_start__ = .;
        . = . + __Minimum_Stack_Size__;
        . = ALIGN(4);
        __checkstack_end__ = .;
    } > ram

    
    /* Rest is debugging info */
    
    /* remove the debugging information from the standard libraries * /
    /DISCARD/ :
    {
    
     libc.a ( * )
     libm.a ( * )
     libgcc.a ( * )
     
    }

    /* Stabs debugging sections.  */
    .stab          0 : { *(.stab) }
    .stabstr       0 : { *(.stabstr) }
    .stab.excl     0 : { *(.stab.excl) }
    .stab.exclstr  0 : { *(.stab.exclstr) }
    .stab.index    0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment       0 : { *(.comment) }
    /* DWARF debug sections.
       Symbols in the DWARF debugging sections are relative to the beginning
       of the section so we begin them at 0.  */
    /* DWARF 1 */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }
    /* GNU DWARF 1 extensions */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }
    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    /* DWARF 2 */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }
}
