#ifndef __FLEX_SPI_MEM_DEFINES_H
#define __FLEX_SPI_MEM_DEFINES_H

#include "cr_section_macros.h"
#include "fsl_flexspi.h"



#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL        7
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST          13
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD     0
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS         1
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE        2
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR        3
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE 6
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD   4
#define NOR_CMD_LUT_SEQ_IDX_READID             8
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG     9
#define NOR_CMD_LUT_SEQ_IDX_ENTERQPI           10
#define NOR_CMD_LUT_SEQ_IDX_EXITQPI            11
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG      12
#define NOR_CMD_LUT_SEQ_IDX_ERASECHIP          5

#define CUSTOM_LUT_LENGTH            64U
#define FLASH_QUAD_ENABLE            0x40U
#define FLASH_BUSY_STATUS_POL        1U
#define FLASH_BUSY_STATUS_OFFSET     0U
#define FLASH_DUMMY_CYCLES           9U
#define BURST_LEGNTH_REG_SHIFT       0U
#define WRAP_ENABLE_REG_SHIFT        2U
#define DUMMY_CYCLES_REG_SHIFT       3U
#define RESET_PIN_SELECTED_REG_SHIFT 7U


extern status_t flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address);
extern status_t flexspi_nor_flash_page_program(FLEXSPI_Type *base, uint32_t dstAddr, const uint32_t *src);
extern status_t flexspi_nor_get_vendor_id(FLEXSPI_Type *base, uint8_t *vendorId);
extern status_t flexspi_nor_enable_quad_mode(FLEXSPI_Type *base);
extern status_t flexspi_nor_erase_chip(FLEXSPI_Type *base);
extern void flexspi_nor_flash_init(FLEXSPI_Type *base);


#define FLASH_SIZE						0x04000000
#define FLASH_PAGE_SIZE                 256
#define SETTINGS_LOCATION_OFFSET		0x01F00000		// this is offset from base address of 0x3000 0000 (FLASH SPACE)
#define SECTOR_SIZE                     0x1000 /* 4K */
#define SETTINGS_SECTOR                  ((SETTINGS_LOCATION_OFFSET) / (SECTOR_SIZE))
#define EXAMPLE_FLEXSPI_CLOCK           kCLOCK_Flexspi1
#define FLASH_PORT                      kFLEXSPI_PortA1
#define EXAMPLE_FLEXSPI_RX_SAMPLE_CLOCK kFLEXSPI_ReadSampleClkLoopbackFromDqsPad

void BOARD_InitHardware(void);
#define CACHE_MAINTAIN 1

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*${variable:start}*/
#if (defined CACHE_MAINTAIN) && (CACHE_MAINTAIN == 1)
typedef struct _flexspi_cache_status
{
    volatile bool DCacheEnableFlag;
    volatile bool ICacheEnableFlag;
} flexspi_cache_status_t;
#endif
/*${variable:end}*/


static inline void flexspi1_clock_init(void)
{
    /*Clock setting for flexspi1*/
    CLOCK_SetRootClockDiv(kCLOCK_Root_Flexspi1, 0x02);//0x06);  //THis used to be = 2);  TS 2023 05 23
    CLOCK_SetRootClockMux(kCLOCK_Root_Flexspi1, 0);
}
static inline void flexspi2_clock_init(void)
{
    /*Clock setting for flexspi1*/
    CLOCK_SetRootClockDiv(kCLOCK_Root_Flexspi2, 2);
    CLOCK_SetRootClockMux(kCLOCK_Root_Flexspi2, 0);
}

#endif
