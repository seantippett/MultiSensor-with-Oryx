/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008-2013 Code Red Technologies Ltd,
 * Copyright 2015, 2018-2019 NXP
 * (c) NXP Semiconductors 2013-2023
 * Generated linker script file for MIMXRT1176xxxxx
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.7.0 [Build 9198] [2023-01-17] on Jun. 1, 2023, 9:20:01 p.m.
 */

INCLUDE "multisensor_oryx_freertos_cm7_Debug_library.ld"
INCLUDE "multisensor_oryx_freertos_cm7_Debug_memory.ld"

ENTRY(ResetISR_BL)

SECTIONS
{
     .text_Flash2 : ALIGN(4)
    {
       FILL(0xff)
        *(.text_Flash2) /* for compatibility with previous releases */
        *(.text_BOARD_FLASH) /* for compatibility with previous releases */
        *(.text.$Flash2)
        *(.text.$BOARD_FLASH)
        *(.text_Flash2.*) /* for compatibility with previous releases */
        *(.text_BOARD_FLASH.*) /* for compatibility with previous releases */
        *(.text.$Flash2.*)
        *(.text.$BOARD_FLASH.*)
        *(.rodata.$Flash2)
        *(.rodata.$BOARD_FLASH)
        *(.rodata.$Flash2.*)
        *(.rodata.$BOARD_FLASH.*)            } > BOARD_FLASH

    /* Image Vector Table and Boot Data for booting from external flash */
    .boot_hdr : ALIGN(4)
    {
        FILL(0xff)
        . = 0x400 ;
        __boot_hdr_start__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.conf))
        . = 0x1000 ;
        __boot_hdr_ivt_loadaddr__  =  ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.ivt))
        . = 0x1020 ;
        __boot_hdr_boot_data_loadaddr__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.boot_data))
        . = 0x1030 ;
        __boot_hdr_dcd_loadaddr__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.dcd_data))
        __boot_hdr_end__ = ABSOLUTE(.) ;
        . = 0x2000 ;									/* This 0x2000 here moves the next section of memory over to 0x2000 space.  I tried avoiding this, but had trouble.  Not sure why it's important */
    } >BOARD_FLASH_BL_AREA

     .text_Flash2 : ALIGN(4)
    {													
       FILL(0xff)
       	. = ALIGN(4) ;
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))							/* The ISR vectors go here. */
		. = ALIGN(4) ;
        *(.after_vectors*)								/* The 'after_vectors' section is where the startup code sits.  It's in the BL section because it never changes. */
		. = ALIGN(4) ;	
	} >BOARD_FLASH_BL_AREA
	
	.text_FlashMain : ALIGN(4)
	{
		FILL(0xff)
		. = ALIGN(4) ;
		
		
        *(.text_Flash2) /* for compatibility with previous releases */
        *(.text_BOARD_FLASH_BL_AREA) /* for compatibility with previous releases */
        
        *(.text.$Flash2)
        *(.text.$BOARD_FLASH_BL_AREA)
        *(.text_Flash2.*) /* for compatibility with previous releases */
        *(.text_BOARD_FLASH_BL_AREA.*) /* for compatibility with previous releases */
        *(.text.$Flash2.*)
        *(.text.$BOARD_FLASH_BL_AREA.*)
        *(.rodata.$Flash2)
        *(.rodata.$BOARD_FLASH_BL_AREA)
        *(.rodata.$Flash2.*)
        *(.rodata.$BOARD_FLASH_BL_AREA.*)       

		. = ALIGN(4) ;        
        __data_section_table_BL = .;
         LONG(LOADADDR(.data_RAM5));    
         LONG(    ADDR(.data_RAM5)); 	  
         LONG(  SIZEOF(.data_RAM5));	  
        __data_section_table_end_BL = .;  
               	. = ALIGN(4) ;   
        __bss_section_table_BL = .;     
        LONG(    ADDR(.bss_RAM5));    
        LONG(  SIZEOF(.bss_RAM5));    
        __bss_section_table_end_BL = .;     

	*(EXCLUDE_FILE(*ramFunctions_BL.o) .text*)
        *(.rodata .rodata.* .constdata .constdata.*)
        . = ALIGN(4);
	} > BOARD_FLASH




    /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        LONG(LOADADDR(.data_RAM3));
        LONG(    ADDR(.data_RAM3));
        LONG(  SIZEOF(.data_RAM3));
        LONG(LOADADDR(.data_RAM4));
        LONG(    ADDR(.data_RAM4));
        LONG(  SIZEOF(.data_RAM4));
/*          LONG(LOADADDR(.data_RAM5)); /*    */
/*          LONG(    ADDR(.data_RAM5)); /*	  */
/*          LONG(  SIZEOF(.data_RAM5)); /*	  */
        LONG(LOADADDR(.data_RAM6));
        LONG(    ADDR(.data_RAM6));
        LONG(  SIZEOF(.data_RAM6));
        LONG(LOADADDR(.data_RAM7));
        LONG(    ADDR(.data_RAM7));
        LONG(  SIZEOF(.data_RAM7));
        LONG(LOADADDR(.data_RAM8));
        LONG(    ADDR(.data_RAM8));
        LONG(  SIZEOF(.data_RAM8));
        LONG(LOADADDR(.data_RAM9));
        LONG(    ADDR(.data_RAM9));
        LONG(  SIZEOF(.data_RAM9));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        LONG(    ADDR(.bss_RAM3));
        LONG(  SIZEOF(.bss_RAM3));
        LONG(    ADDR(.bss_RAM4));
        LONG(  SIZEOF(.bss_RAM4));
/*        LONG(    ADDR(.bss_RAM5));    /*  */
 /*       LONG(  SIZEOF(.bss_RAM5));    /*  */
        LONG(    ADDR(.bss_RAM6));
        LONG(  SIZEOF(.bss_RAM6));
        LONG(    ADDR(.bss_RAM7));
        LONG(  SIZEOF(.bss_RAM7));
        LONG(    ADDR(.bss_RAM8));
        LONG(  SIZEOF(.bss_RAM8));
        LONG(    ADDR(.bss_RAM9));
        LONG(  SIZEOF(.bss_RAM9));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */


        *(EXCLUDE_FILE(*mflash_drv.o *fsl_flexspi.o) .text*)
               KEEP(*freertos*/tasks.o(.rodata*)) /* FreeRTOS Debug Config */
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > BOARD_FLASH
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > BOARD_FLASH

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > BOARD_FLASH
 
    _etext = .;
        
    /* DATA section for NCACHE_REGION */

    .data_RAM2 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM2 = .) ;
        PROVIDE(__start_data_NCACHE_REGION = .) ;
        *(.ramfunc.$RAM2)
        *(.ramfunc.$NCACHE_REGION)
       *(NonCacheable.init)
        *(.data.$RAM2)
        *(.data.$NCACHE_REGION)
        *(.data.$RAM2.*)
        *(.data.$NCACHE_REGION.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM2 = .) ;
        PROVIDE(__end_data_NCACHE_REGION = .) ;
     } > NCACHE_REGION AT>BOARD_FLASH

    /* DATA section for SRAM_DTC_cm7 */

    .data_RAM3 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM3 = .) ;
        PROVIDE(__start_data_SRAM_DTC_cm7 = .) ;
        *(.ramfunc.$RAM3)
        *(.ramfunc.$SRAM_DTC_cm7)
       "./lwip/*" (.data*)
        *(.data.$RAM3)
        *(.data.$SRAM_DTC_cm7)
        *(.data.$RAM3.*)
        *(.data.$SRAM_DTC_cm7.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM3 = .) ;
        PROVIDE(__end_data_SRAM_DTC_cm7 = .) ;
     } > SRAM_DTC_cm7 AT>BOARD_FLASH

    /* DATA section for SRAM_ITC_cm7 */

    .data_RAM4 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM4 = .) ;
        PROVIDE(__start_data_SRAM_ITC_cm7 = .) ;
        *(.ramfunc.$RAM4)
        *(.ramfunc.$SRAM_ITC_cm7)
       KEEP(*(CodeQuickAccess))
        *(.data.$RAM4)
        *(.data.$SRAM_ITC_cm7)
        *(.data.$RAM4.*)
        *(.data.$SRAM_ITC_cm7.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM4 = .) ;
        PROVIDE(__end_data_SRAM_ITC_cm7 = .) ;
     } > SRAM_ITC_cm7 AT>BOARD_FLASH

    /* DATA section for SRAM_ITC_cm7_BL */

    .data_RAM5 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM5 = .) ;
        PROVIDE(__start_data_SRAM_ITC_cm7_BL = .) ;
        *(.ramfunc.$RAM5)
        *(.ramfunc.$SRAM_ITC_cm7_BL)
        *(.data.$RAM5)
        *(.data.$SRAM_ITC_cm7_BL)
        *(.data.$RAM5.*)
        *(.data.$SRAM_ITC_cm7_BL.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM5 = .) ;
        PROVIDE(__end_data_SRAM_ITC_cm7_BL = .) ;
     } > SRAM_ITC_cm7_BL AT>BOARD_FLASH_BL_AREA

    /* DATA section for SRAM_OC1 */

    .data_RAM6 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM6 = .) ;
        PROVIDE(__start_data_SRAM_OC1 = .) ;
        *(.ramfunc.$RAM6)
        *(.ramfunc.$SRAM_OC1)
        *(.data.$RAM6)
        *(.data.$SRAM_OC1)
        *(.data.$RAM6.*)
        *(.data.$SRAM_OC1.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM6 = .) ;
        PROVIDE(__end_data_SRAM_OC1 = .) ;
     } > SRAM_OC1 AT>BOARD_FLASH

    /* DATA section for SRAM_OC2 */

    .data_RAM7 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM7 = .) ;
        PROVIDE(__start_data_SRAM_OC2 = .) ;
        *(.ramfunc.$RAM7)
        *(.ramfunc.$SRAM_OC2)
        *(.data.$RAM7)
        *(.data.$SRAM_OC2)
        *(.data.$RAM7.*)
        *(.data.$SRAM_OC2.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM7 = .) ;
        PROVIDE(__end_data_SRAM_OC2 = .) ;
     } > SRAM_OC2 AT>BOARD_FLASH

    /* DATA section for SRAM_OC_ECC1 */

    .data_RAM8 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM8 = .) ;
        PROVIDE(__start_data_SRAM_OC_ECC1 = .) ;
        *(.ramfunc.$RAM8)
        *(.ramfunc.$SRAM_OC_ECC1)
        *(.data.$RAM8)
        *(.data.$SRAM_OC_ECC1)
        *(.data.$RAM8.*)
        *(.data.$SRAM_OC_ECC1.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM8 = .) ;
        PROVIDE(__end_data_SRAM_OC_ECC1 = .) ;
     } > SRAM_OC_ECC1 AT>BOARD_FLASH

    /* DATA section for SRAM_OC_ECC2 */

    .data_RAM9 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM9 = .) ;
        PROVIDE(__start_data_SRAM_OC_ECC2 = .) ;
        *(.ramfunc.$RAM9)
        *(.ramfunc.$SRAM_OC_ECC2)
        *(.data.$RAM9)
        *(.data.$SRAM_OC_ECC2)
        *(.data.$RAM9.*)
        *(.data.$SRAM_OC_ECC2.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM9 = .) ;
        PROVIDE(__end_data_SRAM_OC_ECC2 = .) ;
     } > SRAM_OC_ECC2 AT>BOARD_FLASH

    /* ORYX */
    /* NOINIT section for SRAM_OC_NONCACHEABLE */
    .noinit_RAM10_non_cacheable (NOLOAD) :
    {
       . = ALIGN(4) ;
       *(.ram_no_cache)
       . = ALIGN(4) ;
    } > SRAM_OC_NONCACHEABLE

    /* MAIN DATA SECTION */
    .uninit_RESERVED (NOLOAD) : ALIGN(4)
    {
        _start_uninit_RESERVED = .;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > BOARD_SDRAM AT> BOARD_SDRAM

    /* Main DATA section (BOARD_SDRAM) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_RAM = .) ;
       PROVIDE(__start_data_BOARD_SDRAM = .) ;
       *(vtable)
       *(.ramfunc*)
       KEEP(*(CodeQuickAccess))
       KEEP(*(DataQuickAccess))
       *(RamFunction)
        *(.data*)        
       . = ALIGN(4) ;
       _edata = . ;
       PROVIDE(__end_data_RAM = .) ;
       PROVIDE(__end_data_BOARD_SDRAM = .) ;
    } > BOARD_SDRAM AT>BOARD_FLASH

    /* BSS section for NCACHE_REGION */
    .bss_RAM2 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM2 = .) ;
       PROVIDE(__start_bss_NCACHE_REGION = .) ;
       *(NonCacheable)
       *(.bss.$RAM2)
       *(.bss.$NCACHE_REGION)
       *(.bss.$RAM2.*)
       *(.bss.$NCACHE_REGION.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
       PROVIDE(__end_bss_NCACHE_REGION = .) ;
    } > NCACHE_REGION AT> NCACHE_REGION

    /* BSS section for SRAM_DTC_cm7 */
    .bss_RAM3 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM3 = .) ;
       PROVIDE(__start_bss_SRAM_DTC_cm7 = .) ;
       "./lwip/*" (.bss*)

       /* ORYX */
       "./oryx/cyclone_crypto/hardware/mimxrt1170/*" (.bss*)

       *(.bss.$RAM3)
       *(.bss.$SRAM_DTC_cm7)
       *(.bss.$RAM3.*)
       *(.bss.$SRAM_DTC_cm7.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM3 = .) ;
       PROVIDE(__end_bss_SRAM_DTC_cm7 = .) ;
    } > SRAM_DTC_cm7 AT> SRAM_DTC_cm7

    /* BSS section for SRAM_ITC_cm7 */
    .bss_RAM4 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM4 = .) ;
       PROVIDE(__start_bss_SRAM_ITC_cm7 = .) ;
       *(.bss.$RAM4)
       *(.bss.$SRAM_ITC_cm7)
       *(.bss.$RAM4.*)
       *(.bss.$SRAM_ITC_cm7.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM4 = .) ;
       PROVIDE(__end_bss_SRAM_ITC_cm7 = .) ;
    } > SRAM_ITC_cm7 AT> SRAM_ITC_cm7

    /* BSS section for SRAM_ITC_cm7_BL */
    .bss_RAM5 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM5 = .) ;
       PROVIDE(__start_bss_SRAM_ITC_cm7_BL = .) ;
       *(.bss.$RAM5)
       *(.bss.$SRAM_ITC_cm7_BL)
       *(.bss.$RAM5.*)
       *(.bss.$SRAM_ITC_cm7_BL.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM5 = .) ;
       PROVIDE(__end_bss_SRAM_ITC_cm7_BL = .) ;
    } > SRAM_ITC_cm7_BL AT> SRAM_ITC_cm7_BL

    /* BSS section for SRAM_OC1 */
    .bss_RAM6 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM6 = .) ;
       PROVIDE(__start_bss_SRAM_OC1 = .) ;
       *(.bss.$RAM6)
       *(.bss.$SRAM_OC1)
       *(.bss.$RAM6.*)
       *(.bss.$SRAM_OC1.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM6 = .) ;
       PROVIDE(__end_bss_SRAM_OC1 = .) ;
    } > SRAM_OC1 AT> SRAM_OC1

    /* BSS section for SRAM_OC2 */
    .bss_RAM7 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM7 = .) ;
       PROVIDE(__start_bss_SRAM_OC2 = .) ;
       *(.bss.$RAM7)
       *(.bss.$SRAM_OC2)
       *(.bss.$RAM7.*)
       *(.bss.$SRAM_OC2.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM7 = .) ;
       PROVIDE(__end_bss_SRAM_OC2 = .) ;
    } > SRAM_OC2 AT> SRAM_OC2

    /* BSS section for SRAM_OC_ECC1 */
    .bss_RAM8 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM8 = .) ;
       PROVIDE(__start_bss_SRAM_OC_ECC1 = .) ;
       *(.bss.$RAM8)
       *(.bss.$SRAM_OC_ECC1)
       *(.bss.$RAM8.*)
       *(.bss.$SRAM_OC_ECC1.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM8 = .) ;
       PROVIDE(__end_bss_SRAM_OC_ECC1 = .) ;
    } > SRAM_OC_ECC1 AT> SRAM_OC_ECC1

    /* BSS section for SRAM_OC_ECC2 */
    .bss_RAM9 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM9 = .) ;
       PROVIDE(__start_bss_SRAM_OC_ECC2 = .) ;
       *(.bss.$RAM9)
       *(.bss.$SRAM_OC_ECC2)
       *(.bss.$RAM9.*)
       *(.bss.$SRAM_OC_ECC2.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM9 = .) ;
       PROVIDE(__end_bss_SRAM_OC_ECC2 = .) ;
    } > SRAM_OC_ECC2 AT> SRAM_OC_ECC2

    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        PROVIDE(__start_bss_RAM = .) ;
        PROVIDE(__start_bss_BOARD_SDRAM = .) ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(__end_bss_RAM = .) ;
        PROVIDE(__end_bss_BOARD_SDRAM = .) ;
        PROVIDE(end = .);
    } > BOARD_SDRAM AT> BOARD_SDRAM

    /* NOINIT section for NCACHE_REGION */
    .noinit_RAM2 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM2 = .) ;
       PROVIDE(__start_noinit_NCACHE_REGION = .) ;
       *(.noinit.$RAM2)
       *(.noinit.$NCACHE_REGION)
       *(.noinit.$RAM2.*)
       *(.noinit.$NCACHE_REGION.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM2 = .) ;
       PROVIDE(__end_noinit_NCACHE_REGION = .) ;
    } > NCACHE_REGION AT> NCACHE_REGION

    /* NOINIT section for SRAM_DTC_cm7 */
    .noinit_RAM3 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM3 = .) ;
       PROVIDE(__start_noinit_SRAM_DTC_cm7 = .) ;
       *(.noinit.$RAM3)
       *(.noinit.$SRAM_DTC_cm7)
       *(.noinit.$RAM3.*)
       *(.noinit.$SRAM_DTC_cm7.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM3 = .) ;
       PROVIDE(__end_noinit_SRAM_DTC_cm7 = .) ;
    } > SRAM_DTC_cm7 AT> SRAM_DTC_cm7

    /* NOINIT section for SRAM_ITC_cm7 */
    .noinit_RAM4 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM4 = .) ;
       PROVIDE(__start_noinit_SRAM_ITC_cm7 = .) ;
       *(.noinit.$RAM4)
       *(.noinit.$SRAM_ITC_cm7)
       *(.noinit.$RAM4.*)
       *(.noinit.$SRAM_ITC_cm7.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM4 = .) ;
       PROVIDE(__end_noinit_SRAM_ITC_cm7 = .) ;
    } > SRAM_ITC_cm7 AT> SRAM_ITC_cm7

    /* NOINIT section for SRAM_ITC_cm7_BL */
    .noinit_RAM5 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM5 = .) ;
       PROVIDE(__start_noinit_SRAM_ITC_cm7_BL = .) ;
       *(.noinit.$RAM5)
       *(.noinit.$SRAM_ITC_cm7_BL)
       *(.noinit.$RAM5.*)
       *(.noinit.$SRAM_ITC_cm7_BL.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM5 = .) ;
       PROVIDE(__end_noinit_SRAM_ITC_cm7_BL = .) ;
    } > SRAM_ITC_cm7_BL AT> SRAM_ITC_cm7_BL

    /* NOINIT section for SRAM_OC1 */
    .noinit_RAM6 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM6 = .) ;
       PROVIDE(__start_noinit_SRAM_OC1 = .) ;
       *(.noinit.$RAM6)
       *(.noinit.$SRAM_OC1)
       *(.noinit.$RAM6.*)
       *(.noinit.$SRAM_OC1.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM6 = .) ;
       PROVIDE(__end_noinit_SRAM_OC1 = .) ;
    } > SRAM_OC1 AT> SRAM_OC1

    /* NOINIT section for SRAM_OC2 */
    .noinit_RAM7 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM7 = .) ;
       PROVIDE(__start_noinit_SRAM_OC2 = .) ;
       *(.noinit.$RAM7)
       *(.noinit.$SRAM_OC2)
       *(.noinit.$RAM7.*)
       *(.noinit.$SRAM_OC2.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM7 = .) ;
       PROVIDE(__end_noinit_SRAM_OC2 = .) ;
    } > SRAM_OC2 AT> SRAM_OC2

    /* NOINIT section for SRAM_OC_ECC1 */
    .noinit_RAM8 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM8 = .) ;
       PROVIDE(__start_noinit_SRAM_OC_ECC1 = .) ;
       *(.noinit.$RAM8)
       *(.noinit.$SRAM_OC_ECC1)
       *(.noinit.$RAM8.*)
       *(.noinit.$SRAM_OC_ECC1.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM8 = .) ;
       PROVIDE(__end_noinit_SRAM_OC_ECC1 = .) ;
    } > SRAM_OC_ECC1 AT> SRAM_OC_ECC1

    /* NOINIT section for SRAM_OC_ECC2 */
    .noinit_RAM9 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM9 = .) ;
       PROVIDE(__start_noinit_SRAM_OC_ECC2 = .) ;
       *(.noinit.$RAM9)
       *(.noinit.$SRAM_OC_ECC2)
       *(.noinit.$RAM9.*)
       *(.noinit.$SRAM_OC_ECC2.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM9 = .) ;
       PROVIDE(__end_noinit_SRAM_OC_ECC2 = .) ;
    } > SRAM_OC_ECC2 AT> SRAM_OC_ECC2

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_RAM = .) ;
        PROVIDE(__start_noinit_BOARD_SDRAM = .) ;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
       PROVIDE(__end_noinit_RAM = .) ;
       PROVIDE(__end_noinit_BOARD_SDRAM = .) ;        
    } > BOARD_SDRAM AT> BOARD_SDRAM

    /* Reserve and place Heap within memory map */
    _HeapSize = 0x400000;			/* 0x40000 */
    .heap (NOLOAD) :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > BOARD_SDRAM

     _StackSize = 0x2000;
     /* Reserve space in memory for Stack */
    .heap2stackfill (NOLOAD) :
    {
        . += _StackSize;
    } > BOARD_SDRAM
    /* Locate actual Stack in memory map */
    .stack ORIGIN(BOARD_SDRAM) + LENGTH(BOARD_SDRAM) - _StackSize - 0 (NOLOAD) :  ALIGN(4)
    {
        _vStackBase = .;
        . = ALIGN(4);
        _vStackTop = . + _StackSize;
    } > BOARD_SDRAM

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
    /* Provide symbols for MIMXRT1170 parts for boot header generation code
     * to set image to be plain load image or XIP.
     * Config : Plain load image = false
     */
    _boot_loadaddr = ORIGIN(BOARD_FLASH_BL_AREA);
    _boot_size = LENGTH(BOARD_FLASH_BL_AREA);
}