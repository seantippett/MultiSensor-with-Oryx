#include <string.h>
#include <math.h>
#include <simpleFlashSupport.h>

#include "app_audio.h"
#include "fsl_pdm.h"
#include "fsl_pdm_edma.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_gpt.h"
#include "fsl_anatop_ai.h"
#include "fsl_lpspi.h"
#include "fsl_lpi2c_freertos.h"
#include "app_shared.h"
//#include "app_shield.h"

//#include "McuWait.h"
//#include "McuRTOS.h"

#include "core/net.h"

#include "board.h"
#include "app_microwave.h"
#include "IMD200x.h"
#include "app_jack.h"

#include "app_board.h"
#include "app_pir.h"
#include "app_config.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "flexSPI_MEM_DEFINES.h"

extern const struct STR_boardSettingTable boardSettingTable;
#include "flexSPI_MEM_DEFINES.h"

#define EXAMPLE_FLEXSPI FLEXSPI1

////////
#include "cr_section_macros.h"
#include "fsl_flexspi.h"
#include "flexSPI_MEM_DEFINES.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "app_config.h"
union U_boardSettings {
	uint8_t raw[1024];
	struct Proc_config_S proc_config;
};

struct STR_boardSettingTable {
	union U_boardSettings defaultSettings;
	union U_boardSettings currentSettings;
	union U_boardSettings backupSettings;
};

struct Proc_config_S Proc_config = {
		.detect = {
				.Event_Count = 1,
				.Event_Window = 10,
				.Alarm_Window = 30,
				.Alarm_Hold_Time = 1,

				.HPF_val = 3,
				.LPF_val = 0,
				.SupMode = 3,
				.Event_Threshold = 75,		// T.S. This device is much heavier than LM100... So we dial down sensitivity to compensate for the mechanical dampening.  T.S. 2022 07 24

				.Lighting_Mode = 1,		//define Light behavior
				.Brightness_Nominal = 255,	//LED brightness in lighting mode
				.Brightness_Alarm = 255,		//LED brightness in alarm, if used
				.Zone_Seg = 0				//store zone info which may be used in broadcast messages
		},
		.RF = {
				.channel = 1,
				.MAC_address = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}
		}
};



//__RODATA(BOARD_FLASH_SETTINGS)
const struct STR_boardSettingTable boardSettingTable = {
		.defaultSettings.proc_config.detect = {
				.Event_Count = 1,
				.Event_Window = 10,
				.Alarm_Window = 30,
				.Alarm_Hold_Time = 1,

				.HPF_val = 3,
				.LPF_val = 0,
				.SupMode = 3,
				.Event_Threshold = 75,		// T.S. This device is much heavier than LM100... So we dial down sensitivity to compensate for the mechanical dampening.  T.S. 2022 07 24

				.Lighting_Mode = 1,		//define Light behavior
				.Brightness_Nominal = 255,	//LED brightness in lighting mode
				.Brightness_Alarm = 255,		//LED brightness in alarm, if used
				.Zone_Seg = 0				//store zone info which may be used in broadcast messages
		},
		.defaultSettings.proc_config.RF = {
				.channel = 1,
				.MAC_address = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}
		}
};

flexspi_device_config_t deviceconfig = {
    .flexspiRootClk       = 12000000,
    .flashSize            = FLASH_SIZE,
    .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval           = 2,
    .CSHoldTime           = 3,
    .CSSetupTime          = 3,
    .dataValidTime        = 0,
    .columnspace          = 0,
    .enableWordAddress    = 0,
    .AWRSeqIndex          = 0,
    .AWRSeqNumber         = 0,
    .ARDSeqIndex          = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD,
    .ARDSeqNumber         = 1,
    .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};


#define CUSTOM_LUT_LENGTH            64U
#define FLASH_QUAD_ENABLE            0x40U
#define FLASH_BUSY_STATUS_POL        1U
#define FLASH_BUSY_STATUS_OFFSET     0U
#define FLASH_DUMMY_CYCLES           9U
#define BURST_LEGNTH_REG_SHIFT       0U
#define WRAP_ENABLE_REG_SHIFT        2U
#define DUMMY_CYCLES_REG_SHIFT       3U
#define RESET_PIN_SELECTED_REG_SHIFT 7U



const uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
    /* Normal read mode -SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x03, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Fast read mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0B, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Fast read quad mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x06, kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x04),

    /* Read extend parameters */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x81, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Write Enable */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Erase Sector  */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xD7, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

    /* Page Program - single mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x02, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Page Program - quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read ID */
    [4 * NOR_CMD_LUT_SEQ_IDX_READID] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enable Quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x01, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enter QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_ENTERQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x35, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Exit QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_EXITQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_4PAD, 0xF5, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read status register */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Erase whole chip */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASECHIP] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xC7, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),
};


//https://www.1024sky.cn/blog/article/53420
//https://www.cnblogs.com/henjay724/p/14479276.html

#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG 9
#define NOR_CMD_LUT_SEQ_IDX_SETREADPARAM   14
status_t flexspi_nor_set_flash_register(FLEXSPI_Type *base, uint32_t seqIdx, uint32_t regValue);

__SECTION(data,SRAM_ITC_cm7_BL) uint32_t index ;
__SECTION(data,SRAM_ITC_cm7_BL) uint32_t	SWSPI_clk_Bit;
__SECTION(data,SRAM_ITC_cm7_BL) uint32_t	SWSPI_csn_Bit;
__SECTION(data,SRAM_ITC_cm7_BL) uint32_t	SWSPI_rst_Bit;
__SECTION(data,SRAM_ITC_cm7_BL) uint32_t	SWSPI_wpn_Bit;
__SECTION(data,SRAM_ITC_cm7_BL) uint32_t	SWSPI_mosi_Bit;
__SECTION(data,SRAM_ITC_cm7_BL) uint32_t	SWSPI_miso_Bit;
status_t flexspi_nor_write_enable(FLEXSPI_Type *base, uint32_t baseAddr);
status_t flexspi_nor_wait_bus_busy(FLEXSPI_Type *base);
status_t flexspi_nor_set_flash_register(FLEXSPI_Type *base, uint32_t seqIdx, uint32_t regValue)
{
    flexspi_transfer_t flashXfer;
    status_t status;
    uint32_t writeValue = regValue;

    /* Write enable */
    status = flexspi_nor_write_enable(base, 0);
    if (status != kStatus_Success)
    {
        return status;
    }

    flashXfer.deviceAddress = 0;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = seqIdx;
    flashXfer.data          = &writeValue;
    flashXfer.dataSize      = 1;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_wait_bus_busy(base);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(base);

    return status;
}

//__attribute__ ((__section__(".data_RAM5")))
//void reset_flash(void)
__RAMFUNC(SRAM_ITC_cm7_BL) void reset_flash(void)
{


//    IOMUXC_SetPinMux(
//    	IOMUXC_GPIO_SD_B2_11_GPIO10_IO20,         /* GPIO_LPSR_04 is configured as LPI2C5_SDA */
//        0U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_04 */
	*(uint32_t *)0x400E81e0 = 0xA;

    GPIO10->GDIR |= 0x100000;
    GPIO10->DR_SET = 0x100000;

    for(index = 0; index < 0x00FF0000; index++);

    GPIO10->DR_CLEAR = 0x100000;

    for(index = 0; index < 0x00FF0000; index++);

    GPIO10->DR_SET = 0x100000;

//    IOMUXC_SetPinMux(
//        IOMUXC_GPIO_SD_B2_11_FLEXSPI1_A_DATA03,  /* GPIO_SD_B2_11 is configured as FLEXSPI1_A_DATA03 */
//        1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_11 */
	*(uint32_t *)0x400E81e0 = 0x1;


}
void reset_flash_to_normal(void)
{
 //   __disable_irq();

//    flexspi_nor_flash_init(EXAMPLE_FLEXSPI);

    // Disable quad mode.

    flexspi_nor_set_flash_register(EXAMPLE_FLEXSPI, NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG, 0x00);

    // Set IO3 pin to Reset func
    flexspi_nor_set_flash_register(EXAMPLE_FLEXSPI, NOR_CMD_LUT_SEQ_IDX_SETREADPARAM, 0x80);

    // Drive IO3 to low for at least 1us
    reset_flash();

    // Set back IO3 pin func
    flexspi_nor_set_flash_register(EXAMPLE_FLEXSPI, NOR_CMD_LUT_SEQ_IDX_SETREADPARAM, 0x00);

    // Enter quad mode.
    flexspi_nor_set_flash_register(EXAMPLE_FLEXSPI, NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG, 0x00);

    NVIC_SystemReset();
}

__STATIC_FORCEINLINE void SCB_InvalidateDCache_by_Addr_RAMFUNC (void *addr, int32_t dsize)
{
  #if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ( dsize > 0 ) {
       int32_t op_size = dsize + (((uint32_t)addr) & (__SCB_DCACHE_LINE_SIZE - 1U));
      uint32_t op_addr = (uint32_t)addr /* & ~(__SCB_DCACHE_LINE_SIZE - 1U) */;

      __DSB();

      do {
        SCB->DCIMVAC = op_addr;             /* register accepts only 32byte aligned values, only bits 31..5 are valid */
        op_addr += __SCB_DCACHE_LINE_SIZE;
        op_size -= __SCB_DCACHE_LINE_SIZE;
      } while ( op_size > 0 );

      __DSB();
      __ISB();
    }
  #endif
}
void L1CACHE_InvalidateICacheByRange_RAMFUNC(uint32_t address, uint32_t size_byte)
{
#if (__DCACHE_PRESENT == 1U)
    uint32_t addr      = address & ~((uint32_t)FSL_FEATURE_L1ICACHE_LINESIZE_BYTE - 1U);
    uint32_t align_len = address - addr;
    int32_t size       = (int32_t)size_byte + (int32_t)align_len;

    __DSB();
    while (size > 0)
    {
        SCB->ICIMVAU = addr;
        addr += (uint32_t)FSL_FEATURE_L1ICACHE_LINESIZE_BYTE;
        size -= (int32_t)FSL_FEATURE_L1ICACHE_LINESIZE_BYTE;
    }
    __DSB();
    __ISB();
#endif
}


extern flexspi_device_config_t deviceconfig;
void flexspi_nor_disable_cache(flexspi_cache_status_t *cacheStatus);
void flexspi_nor_enable_cache(flexspi_cache_status_t cacheStatus);
#if(0)
void flexspi_nor_flash_returnToNormal(FLEXSPI_Type *base)
{
    flexspi_config_t config;
    /* To store custom's LUT table in local. */
    uint32_t tempLUT[CUSTOM_LUT_LENGTH] = {0x00U};

#if defined(CACHE_MAINTAIN) && CACHE_MAINTAIN
    flexspi_cache_status_t cacheStatus;
    flexspi_nor_disable_cache(&cacheStatus);
#endif

    /* Copy LUT information from flash region into RAM region, because LUT update maybe corrupt read sequence(LUT[0])
     * and load wrong LUT table from FLASH region. */
//    memcpy(tempLUT, customLUT, sizeof(tempLUT));

//    flexspi_clock_init();

    /*Get FLEXSPI default settings and configure the flexspi. */
    FLEXSPI_GetDefaultConfig(&config);

    /*Set AHB buffer size for reading data through AHB bus. */
    config.ahbConfig.enableAHBPrefetch    = false;
    config.ahbConfig.enableAHBBufferable  = false;
    config.ahbConfig.enableReadAddressOpt = false;
    config.ahbConfig.enableAHBCachable    = false;
    config.rxSampleClock                  = kFLEXSPI_ReadSampleClkLoopbackInternally;
    FLEXSPI_Init(base, &config);

    /* Configure flash settings according to serial flash feature. */
    FLEXSPI_SetFlashConfig(base, &deviceconfig, kFLEXSPI_PortB1);

    /* Update LUT table. */
    FLEXSPI_UpdateLUT(base, 0, tempLUT, CUSTOM_LUT_LENGTH);

    base->MCR0 = 0xFFFF8010;
    base->FLSHCR4 = 0x3;
    base->IPRXFCR = 0x1C;

    flexspi_clock_fullSpeed();


    /* Do software reset. */
//    FLEXSPI_SoftwareReset(base);
 //   FLEXSPI_ClearAhbBuffer(base);
#if defined(CACHE_MAINTAIN) && CACHE_MAINTAIN
    flexspi_nor_enable_cache(cacheStatus);
#endif
}
#endif








//__DATA(SRAM_DTC_cm7) SDK_ALIGN(static uint8_t s_nor_program_buffer[256], 4);
//__DATA(SRAM_DTC_cm7) static uint8_t s_nor_read_buffer[256];
__DATA(SRAM_DTC_cm7) uint32_t i = 0;
__DATA(SRAM_DTC_cm7) status_t status;
__DATA(SRAM_DTC_cm7) uint8_t vendorID = 0;

void loadCurrentSettings(void)
{
	/* Program data buffer should be 4-bytes alignment, which can avoid busfault due to this memory region is configured as
	   Device Memory by MPU. */

#if(0)
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B2_05_FLEXSPI1_A_DQS,    /* GPIO_SD_B2_05 is configured as FLEXSPI1_A_DQS */
        1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_05 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B2_06_FLEXSPI1_A_SS0_B,  /* GPIO_SD_B2_06 is configured as FLEXSPI1_A_SS0_B */
        1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_06 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B2_07_FLEXSPI1_A_SCLK,   /* GPIO_SD_B2_07 is configured as FLEXSPI1_A_SCLK */
        1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_07 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B2_08_FLEXSPI1_A_DATA00,  /* GPIO_SD_B2_08 is configured as FLEXSPI1_A_DATA00 */
        1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_08 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B2_09_FLEXSPI1_A_DATA01,  /* GPIO_SD_B2_09 is configured as FLEXSPI1_A_DATA01 */
        1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_09 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B2_10_FLEXSPI1_A_DATA02,  /* GPIO_SD_B2_10 is configured as FLEXSPI1_A_DATA02 */
        1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_10 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B2_11_FLEXSPI1_A_DATA03,  /* GPIO_SD_B2_11 is configured as FLEXSPI1_A_DATA03 */
        1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_11 */

#endif
#if(0)
	if(boardSettingTable.currentSettings.raw[1023] == 0xFE){
		// current settings are valid.  load those.
		memcpy(&Proc_config, &boardSettingTable.currentSettings.proc_config, sizeof(Proc_config));
		return;

	}

	// current settings are not valid.  check the backup settings
	if(boardSettingTable.backupSettings.raw[1023] == 0xFE){
		memcpy(&Proc_config, &boardSettingTable.backupSettings.proc_config, sizeof(Proc_config));
	}else {
		// nope, going with just the defaults
		memcpy(&Proc_config, &(boardSettingTable.defaultSettings.proc_config), sizeof(Proc_config));
	}

	// our 'currentSettings' were not valid.  So we need to copy what we just loaded in to Current Settings.
#endif

 //   flexspi_nor_flash_init(FLEXSPI1, kFLEXSPI_PortB1);
 //   PRINTF("\r\nFLEXSPI example started!\r\n");
#if(0)

    /* Get vendor ID. */
    status = flexspi_nor_get_vendor_id(FLEXSPI1, &vendorID);
//    status = (*fp1)(FLEXSPI1, &vendorID);
//    if (status != kStatus_Success)
//    {
//        return status;
//    }
//    PRINTF("Vendor ID: 0x%x\r\n", vendorID);
    /* Enter quad mode. */
    status = flexspi_nor_enable_quad_mode(FLEXSPI1);
//    status = (*fp2)(FLEXSPI1);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Erase sectors. */
//    PRINTF("Erasing Serial NOR over FlexSPI...\r\n");
    status = flexspi_nor_flash_erase_sector(FLEXSPI1, SETTINGS_SECTOR * SECTOR_SIZE);
    if (status != kStatus_Success)
    {
//        PRINTF("Erase sector failure !\r\n");
        return -1;
    }

    //    memset(s_nor_program_buffer, 0xFFU, sizeof(s_nor_program_buffer));
    for(i = 0; i < sizeof(s_nor_program_buffer); i++){
    	s_nor_program_buffer[i] = 0xFF;
    }

    SCB_InvalidateDCache_by_Addr_RAMFUNC(FlexSPI1_AMBA_BASE + SETTINGS_SECTOR * SECTOR_SIZE, FLASH_PAGE_SIZE);


//    memcpy(s_nor_read_buffer, (void *)(FlexSPI1_AMBA_BASE + SETTINGS_SECTOR * SECTOR_SIZE),
//           sizeof(s_nor_read_buffer));
    for(i = 0; i < sizeof(s_nor_program_buffer); i++){
    	s_nor_read_buffer[i] = *(uint8_t *)(FlexSPI1_AMBA_BASE + SETTINGS_SECTOR * SECTOR_SIZE + i);
    }



//    if (memcmp(s_nor_program_buffer, s_nor_read_buffer, sizeof(s_nor_program_buffer)))
//    {
//        PRINTF("Erase data -  read out data value incorrect !\r\n ");
//        return -1;
//    }
//    else
//    {
//        PRINTF("Erase data - successfully. \r\n");
//    }

    for(i = 0; i < sizeof(s_nor_program_buffer); i++){
    	if(s_nor_program_buffer[i] != s_nor_read_buffer[i]){
    		//        PRINTF("Erase data -  read out data value incorrect !\r\n ");
    		return -1;
    	}
    }

    for (i = 0; i < 0xFFU; i++)
    {
        s_nor_program_buffer[i] = i;
    }

    status =  flexspi_nor_flash_page_program(FLEXSPI1, SETTINGS_SECTOR * SECTOR_SIZE, (void *)s_nor_program_buffer);
    if (status != kStatus_Success)
    {
//        PRINTF("Page program failure !\r\n");
        return -1;
    }


//    DCACHE_InvalidateByRange(FlexSPI1_AMBA_BASE + SETTINGS_SECTOR * SECTOR_SIZE, FLASH_PAGE_SIZE);
    SCB_InvalidateDCache_by_Addr_RAMFUNC(FlexSPI1_AMBA_BASE + SETTINGS_SECTOR * SECTOR_SIZE, FLASH_PAGE_SIZE);

//    memcpy(s_nor_read_buffer, (void *)(FlexSPI1_AMBA_BASE + SETTINGS_SECTOR * SECTOR_SIZE),
//           sizeof(s_nor_read_buffer));
    for(i = 0; i < sizeof(s_nor_program_buffer); i++){
    	s_nor_read_buffer[i] = *(uint8_t *)(FlexSPI1_AMBA_BASE + SETTINGS_SECTOR * SECTOR_SIZE + i);
    }


    for(i = 0; i < sizeof(s_nor_program_buffer); i++){
    	if(s_nor_program_buffer[i] != s_nor_read_buffer[i]){
    		//        PRINTF("Erase data -  read out data value incorrect !\r\n ");
    		return -1;
    	}
    }


//    if (memcmp(s_nor_read_buffer, s_nor_program_buffer, sizeof(s_nor_program_buffer)) != 0)
//    {
 //       PRINTF("Program data -  read out data value incorrect !\r\n ");
//        return -1;
//    }
//    else
 //   {
//        PRINTF("Program data - successfully. \r\n");
//    }
#endif
//    flexspi_nor_flash_returnToNormal(FLEXSPI1);
    reset_flash_to_normal();
    FLEXSPI1->MCR0 = 0xFFFF8010;
    FLEXSPI1->FLSHCR4 = 0x3;
    FLEXSPI1->IPRXFCR = 0x1C;

//    flexspi_clock_fullSpeed();

//    L1CACHE_InvalidateICacheByRange_RAMFUNC((void *)0x30000000, 0x01F00000);

    return;

}


void SWSPI_SendCmd(uint8_t cmd);
uint32_t	SWSPI_ReadWord(uint8_t cmd, uint32_t address, uint32_t wordLen);
void SWSPI_Write32bitWord(uint8_t cmd, uint32_t address, uint32_t word);
void	SWSPI_WriteByte(uint8_t cmd, uint8_t byte);
uint32_t	SWSPI_ReadByte(uint8_t cmd)	;
void readPage(uint32_t addr, uint8_t *data);
void eraseSector(uint32_t addr);
void writePage(uint32_t addr, uint8_t *data);



__RAMFUNC(SRAM_ITC_cm7_BL) void SWSPI_INIT(uint32_t port, uint32_t resetFlashChip)
{

	// port 0 = PIRMARY   (i.e. chip: U8 - holder of running code)
	// port 1 = SECONDARY (i.e. chip: U9)

	int delay;
	uint32_t vendorID;

	*(uint32_t *)0x400E8100 = 0x16;			// move the DQS pin for FLEXSPI1A.  NOTE it is very important that this is a 0x16 and not a 0x06.  I really need to figure out the true meaning of the 0x1x field.

	if(port == 1){
		*(uint32_t *)0x400E81B4 = 0x1A;		// MUX controllers.
		*(uint32_t *)0x400E81B8 = 0x1A;
		*(uint32_t *)0x400E81BC = 0x1A;
		*(uint32_t *)0x400E81C0 = 0x1A;
		*(uint32_t *)0x400E81C4 = 0x1A;
		*(uint32_t *)0x400E81C8 = 0x1A;		// CSn    - bit14
//		*(uint32_t *)0x400E81A8 = 0x1A;		// CSn    - bit6	-	 THIS REQUIRES A HW mod on the board.  and a change in SWSPI_U9_CSn_BIT

		GPIO10->GDIR |= 0x000200;	// RESETn - bit9
		GPIO10->GDIR |= 0x000400;	// WPn    - bit10
		GPIO10->GDIR &= ~0x000800;	// MISO   - bit11
		GPIO10->GDIR |= 0x001000;	// MOSI   - bit12
		GPIO10->GDIR |= 0x002000;	//SCK	  - bit13
		GPIO10->GDIR |= 0x004000;	// CSn    - bit14
//		GPIO10->GDIR |= 0x000040;	// CSn    - bit6	-	 THIS REQUIRES A HW mod on the board.  and a change in SWSPI_U9_CSn_BIT

		SET_SWSPI_FOR_SECONDARY;
	}else{
			// note that once you do this...  there's no going back. I.e. you're going to need to reset the chip after this.
			// or you're going to need to figure out how to reset the memory controller... may the force be with you.
		*(uint32_t *)0x400E81CC = 0x1A;		// MUX controllers.
		*(uint32_t *)0x400E81D0 = 0x1A;
		*(uint32_t *)0x400E81D4 = 0x1A;
		*(uint32_t *)0x400E81D8 = 0x1A;
		*(uint32_t *)0x400E81DC = 0x1A;
		*(uint32_t *)0x400E81F0 = 0x1A;

		GPIO10->GDIR |=   0x100000;	// RESETn - bit20
		GPIO10->GDIR |=   0x080000;	// WPn    - bit19
		GPIO10->GDIR &=  ~0x040000;	// MISO   - bit18
		GPIO10->GDIR |=   0x020000;	// MOSI   - bit17
		GPIO10->GDIR |=   0x010000;	//SCK	  - bit16
		GPIO10->GDIR |=   0x008000;	// CSn    - bit15

		SET_SWSPI_FOR_PRIMARY;

	}

	if(resetFlashChip == TRUE){
		SWSPI_RESETn_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_WPn_1;
		SWSPI_CLK_0;
		SWSPI_CSn_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_RESETn_1;


		do{
			vendorID = SWSPI_ReadWord(0x90,0,4);		// vendor id only works in 3 byte addressing mode.
		}while(vendorID != 0x189d189d);

		SWSPI_SendCmd(FLASH_CMD_EN4B);
		//	reg = SWSPI_ReadByte(FLASH_CMD_RDBR);				// switch over to 4 byte addressing mode.
	}


}

__RAMFUNC(SRAM_ITC_cm7_BL) void SWSPI_SendCmd(uint8_t cmd)
{
	int i, delay;

	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			// CMD
	for(i = 0;i < 8; i++){
		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
	}
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	SWSPI_CSn_1;
	SWSPI_MOSI_0;
}

__RAMFUNC(SRAM_ITC_cm7_BL) uint32_t	SWSPI_ReadWord(uint8_t cmd, uint32_t address, uint32_t wordLen)	// word length up to 4 bytes.
{
	int i, delay;
	uint32_t rc = 0;

	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			// CMD
	for(i = 0;i < 8; i++){
		rc = rc >> 1;
		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		if(GPIO10->DR & SWSPI_MISO_BIT){
			rc |= 0x8000;
		}
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

	// ADDR
	for(i = 0;i < 32; i++){
		rc = rc >> 1;
		if(address & 0x80000000){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		address <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		if(GPIO10->DR & SWSPI_MISO_BIT){
			rc |= 0x8000;
		}
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}
	rc = 0;

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		// Data
	for(i = 0;i < (wordLen * 8); i++){
		rc = rc << 1;

		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		if(GPIO10->DR & SWSPI_MISO_BIT){
			rc |= 0x0001;
		}
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}
	SWSPI_CSn_1;
	SWSPI_MOSI_0;
	return rc;
}
__RAMFUNC(SRAM_ITC_cm7_BL) void SWSPI_Write32bitWord(uint8_t cmd, uint32_t address, uint32_t word)	// 32Bit word length up to 4 bytes.
{
	int i, delay;

	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			// CMD
	for(i = 0;i < 8; i++){

		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

	// ADDR
	for(i = 0;i < 32; i++){

		if(address & 0x80000000){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		address <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}


	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		// Data
	for(i = 0;i < 32; i++){

		if(word & 0x80000000){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		word <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	SWSPI_CSn_1;
	SWSPI_MOSI_0;

}
__RAMFUNC(SRAM_ITC_cm7_BL) void	SWSPI_WriteByte(uint8_t cmd, uint8_t byte)	// word length up to 4 bytes.
{
	int i, delay;

	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			// CMD
	for(i = 0;i < 8; i++){

		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		// Data
	for(i = 0;i < ( 8); i++){

		if(byte & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		byte <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	SWSPI_CSn_1;
	SWSPI_MOSI_0;

}
__RAMFUNC(SRAM_ITC_cm7_BL) uint32_t	SWSPI_ReadByte(uint8_t cmd)	// word length up to 4 bytes.
{
	int i, delay;
	uint32_t rc = 0;

	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			// CMD
	for(i = 0;i < 8; i++){
		rc = rc >> 1;
		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		if(GPIO10->DR & SWSPI_MISO_BIT){
			rc |= 0x8000;
		}
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}


		// Data
	rc = 0;

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		// Data
	for(i = 0;i < ( 8); i++){
		rc = rc << 1;

		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		if(GPIO10->DR & SWSPI_MISO_BIT){
			rc |= 0x0001;
		}
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}
	SWSPI_CSn_1;
	SWSPI_MOSI_0;
	return rc;
}


__RAMFUNC(SRAM_ITC_cm7_BL) void readPage(uint32_t addr, uint8_t *data)
{		// uses 4 byte addressing
	uint8_t tempByte, cmd = FLASH_CMD_4NORD;
	uint32_t i, delay;
	uint32_t bitIndex;

	// address must be on a page boundry.
	addr = addr & 0xFFFFFF00;


	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			// CMD
	for(i = 0;i < 8; i++){

		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

	// ADDR
	for(i = 0;i < 32; i++){
		if(addr & 0x80000000){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		addr <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		// Data
	for(i = 0; i < 256; i++){
		tempByte = 0;
		for(bitIndex = 0; bitIndex < (8); bitIndex++){
			tempByte = tempByte << 1;

//			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
			SWSPI_CLK_1;
//			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			if(GPIO10->DR & SWSPI_MISO_BIT){
				tempByte |= 0x0001;
			}
//			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
			SWSPI_CLK_0;
//			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		}
		data[i] = tempByte;
	}
	SWSPI_CSn_1;
	SWSPI_MOSI_0;
	return;

}


void readPageXIP(uint32_t addr, uint8_t *data)
{		// uses 4 byte addressing
	uint8_t tempByte, cmd = FLASH_CMD_4NORD;
	uint32_t i, delay;
	uint32_t bitIndex;

	// address must be on a page boundry.
	addr = addr & 0xFFFFFF00;


	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			// CMD
	for(i = 0;i < 8; i++){

		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

	// ADDR
	for(i = 0;i < 32; i++){
		if(addr & 0x80000000){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		addr <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		// Data
	for(i = 0; i < 256; i++){
		tempByte = 0;
		for(bitIndex = 0; bitIndex < (8); bitIndex++){
			tempByte = tempByte << 1;

//			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
			SWSPI_CLK_1;
//			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			if(GPIO10->DR & SWSPI_MISO_BIT){
				tempByte |= 0x0001;
			}
//			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
			SWSPI_CLK_0;
//			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		}
		data[i] = tempByte;
	}
	SWSPI_CSn_1;
	SWSPI_MOSI_0;
	return;

}













__RAMFUNC(SRAM_ITC_cm7_BL) void eraseSector(uint32_t addr){
	// CMD
	uint32_t i, delay;
	uint8_t cmd = FLASH_CMD_SER;
	addr = addr & 0xFFFFF000;							// must be on a sector boundary.

	SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.


	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

	for(i = 0;i < 8; i++){
		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

	// ADDR
	for(i = 0;i < 32; i++){
		if(addr & 0x80000000){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		addr <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}
	SWSPI_CSn_1;
	SWSPI_MOSI_0;
	return;
}
__RAMFUNC(SRAM_ITC_cm7_BL) void erase64KBlock(uint32_t addr){
	// CMD
	uint32_t i, delay;
	uint8_t cmd = FLASH_CMD_BER64;
	addr = addr & 0xFFFFF000;							// must be on a sector boundary.

	SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.


	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

	for(i = 0;i < 8; i++){
		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

	// ADDR
	for(i = 0;i < 32; i++){
		if(addr & 0x80000000){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		addr <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}
	SWSPI_CSn_1;
	SWSPI_MOSI_0;
	return;
}



__RAMFUNC(SRAM_ITC_cm7_BL) void writePage(uint32_t addr, uint8_t *data)
{
	uint8_t tempByte, cmd = FLASH_CMD_4PP;
	uint32_t i, delay;
	uint32_t bitIndex;

	// address must be on a page boundary.
	addr = addr & 0xFFFFFF00;

	SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.

	SWSPI_CSn_0;
	SWSPI_CLK_0;
	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

			// CMD
	for(i = 0;i < 8; i++){

		if(cmd & 0x80){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		cmd <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

	// ADDR
	for(i = 0;i < 32; i++){
		if(addr & 0x80000000){
			SWSPI_MOSI_1;
		}else{
			SWSPI_MOSI_0;
		}
		addr <<= 1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_1;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		SWSPI_CLK_0;
		for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
	}

	for(delay = 0; delay < HALF_BIT_DELAY; delay ++);

		// Data
	for(i = 0; i < 256; i++){
		tempByte = data[i];
		for(bitIndex = 0; bitIndex < (8); bitIndex++){
			if(tempByte & 0x80){
				SWSPI_MOSI_1;
			}else{
				SWSPI_MOSI_0;
			}
			tempByte <<= 1;
			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
			SWSPI_CLK_1;
			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
			SWSPI_CLK_0;
			for(delay = 0; delay < HALF_BIT_DELAY; delay ++);
		}
	}
	SWSPI_CSn_1;
	SWSPI_MOSI_0;
	return;

}







void loadDefaultSettings(void)
{

	memcpy(&Proc_config, &boardSettingTable.defaultSettings.proc_config, sizeof(Proc_config));

}





void loadCurrentSettingsFlexSPI1B(void)
{
//	uint32_t delay;
	uint32_t i;
	union U_FlashStorageEntity storageEntity;

	/* Program data buffer should be 4-bytes alignment, which can avoid busfault due to this memory region is configured as
	   Device Memory by MPU. */
//	uint32_t vendorID;
	uint32_t	reg;
	SWSPI_INIT(1, TRUE);

	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		readPage(FLASH_ADDR__SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}

	if(storageEntity.storageEntityCtrlData == FLASH_CTRL__ValidEntityCode){
		// current settings are valid.  load those.
		memcpy(&configParam, &storageEntity.configData, sizeof(configParam));
		return;
	}

	// current settings are not valid.  check the backup settings
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		readPage(FLASH_ADDR__BACKUP_SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}

	if(storageEntity.storageEntityCtrlData == FLASH_CTRL__ValidEntityCode){
		// settings are valid.  load those.
		memcpy(&configParam, &storageEntity.configData, sizeof(configParam));
	} else {
		// nope, going with just the defaults
		memset(&storageEntity, 0xFF, sizeof(storageEntity));
		memcpy(&storageEntity.configData, &configParam, sizeof(configParam));
	}

	// if we're here, we need to update our main settings.
	eraseSector(FLASH_ADDR__SETTINGS);
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		do{
			reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status
		}while(reg & 0x01);
		writePage(FLASH_ADDR__SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}
	do{
		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);					// Read Status
	}while(reg & 0x01);
	SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__SETTINGS, FLASH_CTRL__ValidEntityCode);			// overwrite the ctrl data to signify that this is good shit.




#if(0)
	if(boardSettingTable.currentSettings.raw[1023] == 0xFE){
		// current settings are valid.  load those.
		memcpy(&Proc_config, &boardSettingTable.currentSettings.proc_config, sizeof(Proc_config));
		return;

	}

	// current settings are not valid.  check the backup settings
	if(boardSettingTable.backupSettings.raw[1023] == 0xFE){
		memcpy(&Proc_config, &boardSettingTable.backupSettings.proc_config, sizeof(Proc_config));
	}else {
		// nope, going with just the defaults
		memcpy(&Proc_config, &(boardSettingTable.defaultSettings.proc_config), sizeof(Proc_config));
	}

	// our 'currentSettings' were not valid.  So we need to copy what we just loaded in to Current Settings.
#endif



#if(0)
	do{
		reg = SWSPI_ReadWord(FLASH_CMD_4NORD,0,4);		// read addr 0.
		SWSPI_WPn_1;

		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status

		SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status

		SWSPI_Write32bitWord(FLASH_CMD_4PP,0,0x01020304);
		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status

		for(delay = 0; delay < PAGE_PROG_DELAY; delay ++);
		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status

		reg = SWSPI_ReadWord(FLASH_CMD_4NORD,0,4);		// read addr 0.


	}while(1);
#endif


}

union U_FlashStorageEntity storageEntity, storageEntityCopy;

void saveNewSettingsToFlash(config_param_t *newSettings)
{
	uint32_t mainValid = 0, backupValid = 0;
	uint8_t reg;


	/* Program data buffer should be 4-bytes alignment, which can avoid busfault due to this memory region is configured as
	   Device Memory by MPU. */
	uint32_t i;


	// check if maindata is valid.
	readPage(FLASH_ADDR__SETTINGS, &(storageEntity.raw[0][0]));
	if((storageEntity.raw[0][0] == FLASH_CTRL__ValidEntityCode_0) &&
			(storageEntity.raw[0][1] == FLASH_CTRL__ValidEntityCode_1) &&
			(storageEntity.raw[0][2] == FLASH_CTRL__ValidEntityCode_2) &&
			(storageEntity.raw[0][3] == FLASH_CTRL__ValidEntityCode_3)) {
		mainValid = TRUE;
	}
	// check if backup data is valid.
	readPage(FLASH_ADDR__BACKUP_SETTINGS, &(storageEntity.raw[0][0]));
	if((storageEntity.raw[0][0] == FLASH_CTRL__ValidEntityCode_0) &&
			(storageEntity.raw[0][1] == FLASH_CTRL__ValidEntityCode_1) &&
			(storageEntity.raw[0][2] == FLASH_CTRL__ValidEntityCode_2) &&
			(storageEntity.raw[0][3] == FLASH_CTRL__ValidEntityCode_3)) {
		backupValid = TRUE;
	}

	// if backup isn't valid... make the copy.
	// if backup doesn't match main... make the copy.  TODO
	if((backupValid == FALSE) && (mainValid == TRUE)){
		for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
			readPage(FLASH_ADDR__SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
		}
		PRINTF("Invalid Backup Copy.  Saving To backup area\r\n");

		storageEntity.storageEntityCtrlData = FLASH_CTRL__EmptyEntityCode;
		memcpy(&storageEntityCopy, &storageEntity, sizeof(storageEntityCopy));

		eraseSector(FLASH_ADDR__BACKUP_SETTINGS);
		for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){

			WAIT_FOR_FLASH_READY;

			writePage(FLASH_ADDR__BACKUP_SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
		}

		WAIT_FOR_FLASH_READY;

		// read back.
		for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
			readPage(FLASH_ADDR__BACKUP_SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
		}
		// compare
		for(i = 0; i < sizeof(config_param_t); i++){
			if(((uint8_t *)(&storageEntity.configData))[i] != ((uint8_t *)(&storageEntityCopy.configData))[i]){
				break;	// mismatch
			}
		}
		if(i == sizeof(config_param_t)){
			SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
//			SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__BACKUP_SETTINGS, FLASH_CTRL__ValidEntityCode);			// overwrite the ctrl data to signify that this is good shit.
			SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__BACKUP_SETTINGS, 0x00A0FFFF);			// overwrite the ctrl data to signify that this is good shit.
			PRINTF("Saving To backup ok\r\n");
		}else{
			PRINTF("Saving To backup area Failed\r\n");
			return;	// abandon ship here... the read back was invalid.
		}
	}
	WAIT_FOR_FLASH_READY;
	// erase settings in main area.
	eraseSector(FLASH_ADDR__SETTINGS);

	// load settings in to the main area.
	memcpy(&storageEntity.configData, newSettings, sizeof(storageEntity.configData) - (((30 * 40)/8) * 3 * 4));
	storageEntity.storageEntityCtrlData = FLASH_CTRL__EmptyEntityCode;
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		WAIT_FOR_FLASH_READY;

		writePage(FLASH_ADDR__SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}

	WAIT_FOR_FLASH_READY;


	// read back.
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		readPage(FLASH_ADDR__SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}
	// compare
	for(i = 0; i < sizeof(config_param_t) - (((30 * 40)/8) * 3 * 4); i++){
		if(((uint8_t *)(&storageEntity.configData))[i] != ((uint8_t *)(newSettings))[i]){
			break;	// mismatch
		}
	}
	if(i == sizeof(config_param_t) - (((30 * 40)/8) * 3 * 4)){
		SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
		SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__SETTINGS, 0x00A0FFFF);			// overwrite the ctrl data to signify that this is good shit.
		PRINTF("Saving To main ok\r\n");
	}else{
		PRINTF("Saving To main Flash area Failed\r\n");
		return;	// abandon ship here... the read back was invalid.
	}


	// erase backup settings.
	eraseSector(FLASH_ADDR__BACKUP_SETTINGS);

	// load setting in to the backup area.
	memcpy(&storageEntity.configData, newSettings, sizeof(storageEntity.configData) - (((30 * 40)/8) * 3 * 4));
	storageEntity.storageEntityCtrlData = FLASH_CTRL__EmptyEntityCode;
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		WAIT_FOR_FLASH_READY;
		writePage(FLASH_ADDR__BACKUP_SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}
	WAIT_FOR_FLASH_READY;

	// read back.
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		readPage(FLASH_ADDR__BACKUP_SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}
	// compare
	for(i = 0; i < sizeof(config_param_t) - (((30 * 40)/8) * 3 * 4); i++){
		if(((uint8_t *)(&storageEntity.configData))[i] != ((uint8_t *)(newSettings))[i]){
			break;	// mismatch
		}
	}
	if(i == sizeof(config_param_t) - (((30 * 40)/8) * 3 * 4)){
		SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
		SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__BACKUP_SETTINGS, 0x00A0FFFF);			// overwrite the ctrl data to signify that this is good shit.
		PRINTF("Saving To backup ok\r\n");
	}else{
		PRINTF("Saving To backup Flash area Failed\r\n");
		return;	// abandon ship here... the read back was invalid.
	}


	PRINTF("done saving Config Data\r\n");


}








void loadFlashSettings(void)
{

	uint32_t i;
	config_param_t defaultConfig;

	union U_FlashStorageEntity storageEntity;
	void (*fp)(uint32_t, uint32_t);
	/* Program data buffer should be 4-bytes alignment, which can avoid busfault due to this memory region is configured as
	   Device Memory by MPU. */

	uint32_t	reg;

	config_init();												// loads a default config in to configParam
	memcpy(&defaultConfig, &configParam, sizeof(defaultConfig));

//	SWSPI_INIT();
	fp = &SWSPI_INIT;
	(*fp)(1,TRUE);

	if(PB_SETTING_RESET_PUSHED){
		for(i = 0; i < 0xFFFFFF; i++){
		}
		if(PB_SETTING_RESET_PUSHED){
			WAIT_FOR_FLASH_READY;
			eraseSector(FLASH_ADDR__SETTINGS);
			WAIT_FOR_FLASH_READY;
			eraseSector(FLASH_ADDR__BACKUP_SETTINGS);
			WAIT_FOR_FLASH_READY;

		}
	}


	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		readPage(FLASH_ADDR__SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}

	if(storageEntity.storageEntityCtrlData == FLASH_CTRL__ValidEntityCode)
	{
		PRINTF("Valid Config in main Flash area.  Loading...\r\n");
		memcpy(&configParam, &storageEntity.configData, sizeof(configParam));
		defaultConfigFlag = 0;

		sanitizeConfig(&configParam, &defaultConfig, 6);
		node_num = configParam.nodeAddress;
		return;
	}
	PRINTF("Invalid config in main flash area\r\n");
	// current settings are not valid.  check the backup settings
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		readPage(FLASH_ADDR__BACKUP_SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}

	if(storageEntity.storageEntityCtrlData == FLASH_CTRL__ValidEntityCode){
		// settings are valid.  load those.
		PRINTF("Backup config is valid.  Using that.\r\n");
		defaultConfigFlag = 0;
		memcpy(&configParam, &storageEntity.configData, sizeof(configParam));
	} else {
		// nope, going with just the defaults
		PRINTF("Backup config is not Valid.  Using Defaults.\r\n");
		defaultConfigFlag = 1;
		memset(&storageEntity, 0xFF, sizeof(storageEntity));
		memcpy(&storageEntity.configData, &configParam, sizeof(configParam));
	}

	sanitizeConfig(&configParam, &defaultConfig, 6);

	PRINTF("Saving to Main Flash area.\r\n");

	// if we're here, we need to update our main settings.
	eraseSector(FLASH_ADDR__SETTINGS);
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){

		WAIT_FOR_FLASH_READY;

		writePage(FLASH_ADDR__SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}

	WAIT_FOR_FLASH_READY;

	// read back.
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY; i++){
		readPage(FLASH_ADDR__SETTINGS + (i * 256), &(storageEntity.raw[i][0]));
	}
	// compare
	for(i = 0; i < sizeof(configParam); i++){
		if(((uint8_t *)(&storageEntity.configData))[i] != ((uint8_t *)(&configParam))[i]){

			PRINTF("Saving Failed.\r\n");
			break;	// mismatch
		}
	}
	if(i == sizeof(configParam)){
		SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
		SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__SETTINGS, 0x00A0FFFF);			// overwrite the ctrl data to signify that this is good shit.
		PRINTF("Saving success!!.\r\n");
	}


	if(defaultConfigFlag){
		// find our Node number based on Unique ID of the processor.
		node_num = AppGetNodeNumber();
		if (!node_num){
			PRINTF("Unique ID -- This looks like a new board with no Flash Config.");
			PRINTF("  Either load Configuration data (via UCM), or place the UID in the lookup table: uid_node_table. \r\n");
			PRINTF("  Node will now use node 0 defaults. \r\n");
		}
	}
	else{
		node_num = configParam.nodeAddress;
	}






#if(0)
	if(boardSettingTable.currentSettings.raw[1023] == 0xFE){
		// current settings are valid.  load those.
		memcpy(&Proc_config, &boardSettingTable.currentSettings.proc_config, sizeof(Proc_config));
		return;

	}

	// current settings are not valid.  check the backup settings
	if(boardSettingTable.backupSettings.raw[1023] == 0xFE){
		memcpy(&Proc_config, &boardSettingTable.backupSettings.proc_config, sizeof(Proc_config));
	}else {
		// nope, going with just the defaults
		memcpy(&Proc_config, &(boardSettingTable.defaultSettings.proc_config), sizeof(Proc_config));
	}

	// our 'currentSettings' were not valid.  So we need to copy what we just loaded in to Current Settings.
#endif



#if(0)
	do{
		reg = SWSPI_ReadWord(FLASH_CMD_4NORD,0,4);		// read addr 0.
		SWSPI_WPn_1;

		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status

		SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status

		SWSPI_Write32bitWord(FLASH_CMD_4PP,0,0x01020304);
		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status

		for(delay = 0; delay < PAGE_PROG_DELAY; delay ++);
		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status

		reg = SWSPI_ReadWord(FLASH_CMD_4NORD,0,4);		// read addr 0.


	}while(1);
#endif


}

extern void reset_flash(void);

extern uint32_t	SWSPI_ReadByte(uint8_t cmd);	// word length up to 4 bytes.
__RAMFUNC(SRAM_ITC_cm7_BL) void readPage(uint32_t addr, uint8_t *data);
void eraseSector(uint32_t addr);
 //

// This will place the 4096 byte buffer into the SRAM_ITC_cm7_BL
__SECTION(data,SRAM_ITC_cm7_BL) uint8_t flashPage[0x1000] ;

#pragma pack(1)
struct STR_FLASH_CTRL_PAGE{
	uint32_t	ctrlField;
	uint32_t	imageSize;
	uint32_t	csum16;
	uint32_t	csum16ImageOnly;
};
extern uint8_t FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START) + 1];
__SECTION(data,SRAM_ITC_cm7_BL) uint32_t BL_Index, BL_pageIndex;
__RAMFUNC(SRAM_ITC_cm7_BL) void checkBootload(void){

	void (*SWSPI_INITfp)(uint32_t, uint32_t);
	void (*readPageFP)(uint32_t addr, uint8_t *data);
	void (*writePageFP)(uint32_t addr, uint8_t *data);
	void (*eraseSectorFP)(uint32_t addr);
	void (*erase64KBlockFP)(uint32_t addr);

	readPageFP = &readPage;
	writePageFP = &writePage;
//	uint8_t    vendorID;
	uint32_t	copyIndex;
	uint32_t	fault;
	uint8_t		reg;
	uint32_t 	flashTransferSource = 0;
	uint32_t	flashImageSize = 0;
	uint16_t	flashImageCSUM16 = 0;
	uint16_t	flashImageCSUM16_imageOnly = 0;
	uint16_t	crc16 = 0;
	eraseSectorFP = &eraseSector;
	erase64KBlockFP = &erase64KBlock;
	SET_SWSPI_FOR_SECONDARY;

	SWSPI_INITfp = &SWSPI_INIT;
	(*SWSPI_INITfp)(1, TRUE);
#if(1)
// read image 1.  Is it valid?
	(*readPageFP)(FLASH_ADDR__CFG_IMAGE_1, flashPage);
	if((flashPage[FLASH_CFG__IMAGE_CTRL_0] == FLASH_CTRL__ValidEntityCode_1) &&
		(flashPage[FLASH_CFG__IMAGE_CTRL_1] == FLASH_CTRL__ValidEntityCode_1) &&
		(flashPage[FLASH_CFG__IMAGE_CTRL_2] == FLASH_CTRL__ValidEntityCode_2) &&
		(flashPage[FLASH_CFG__IMAGE_CTRL_3] == FLASH_CTRL__ValidEntityCode_3)) {

//		load number of bytes in image
		flashImageSize = ((struct STR_FLASH_CTRL_PAGE *)flashPage)->imageSize;
		flashImageCSUM16 = (uint16_t) ((struct STR_FLASH_CTRL_PAGE *)flashPage)->csum16;
		flashImageCSUM16_imageOnly = (uint16_t) ((struct STR_FLASH_CTRL_PAGE *)flashPage)->csum16ImageOnly;


		// READ IMAGE....

		SWSPI_INITfp(1, FALSE);		// use the INIT function to switch over to the secondary flash... but this time we don't need to reset the IC.
		reg = 0;
		for(BL_Index = 0; BL_Index < flashImageSize; BL_Index = BL_Index + FLASH_IMAGE_PAGE_SIZE){
			(*readPageFP)((BL_Index) + FLASH_ADDR__IMAGE_1_START, &(FW_SwapSpace[BL_Index]) );			// we're reading with an offset, because our image is stored in U9 starting at address 0x100000 (that's the start of the whole image, i.e. address 0).
		}

		// VALID?
		crc16 = CRC_INITIAL;
		crc16 = CRCCalcRange(&(FW_SwapSpace[0]), flashImageSize);

		// CSUM match?
		if(crc16 == flashImageCSUM16){
			// set transfer address.
			flashTransferSource = FLASH_ADDR__IMAGE_1_START;
		}else{
			// failed CRC.  Blow away CTRL page so we don't do this again on a crap image.
			eraseSectorFP = &eraseSector;
			(*eraseSectorFP)(FLASH_ADDR__CFG_IMAGE_1);
		}
	}
#endif



	if(flashTransferSource == 0){
#if(0)
		(*readPageFP)(FLASH_ADDR__CFG_IMAGE_2, flashPage);

		if((flashPage[FLASH_CFG__IMAGE_CTRL_0] == FLASH_CTRL__ValidEntityCode_0) &&
			(flashPage[FLASH_CFG__IMAGE_CTRL_1] == FLASH_CTRL__ValidEntityCode_1) &&
			(flashPage[FLASH_CFG__IMAGE_CTRL_2] == FLASH_CTRL__ValidEntityCode_2) &&
			(flashPage[FLASH_CFG__IMAGE_CTRL_3] == FLASH_CTRL__ValidEntityCode_3)) {

			// calc. csum across image 1.


			// valid csum?

			// set transfer address.
			flashTransferSource = FLASH_ADDR__IMAGE_2_START;
		}
#endif
	}


	if(flashTransferSource == 0){
		ABORT_BOOTLOAD;
		return;		// run existing code.
	}

	// INIT main flash.  With a Flash Reset!
	SWSPI_INITfp(0, TRUE);

	// erase main code.
	for(BL_Index = FLASH_MAIN_CODE_SPACE_START; BL_Index < flashImageSize; BL_Index = BL_Index + FLASH_IMAGE_64K_BLOCK_SIZE){
		do{
			reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status
		}while(reg & 0x01);
		(*erase64KBlockFP)(BL_Index);
	}

	SWSPI_INITfp(0, FALSE);		// use the INIT function to switch over to the primary flash... but this time we don't need to reset the IC.

	// transfer flash over to main code space
	for(BL_Index = FLASH_MAIN_CODE_SPACE_START; BL_Index < flashImageSize; BL_Index = BL_Index + FLASH_IMAGE_PAGE_SIZE){

		fault = 0;
		do{

			for(copyIndex = 0; copyIndex < FLASH_IMAGE_PAGE_SIZE; copyIndex++){ flashPage[copyIndex] = FW_SwapSpace[copyIndex + BL_Index];}

			do{
				reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status
			}while(reg & 0x01);
			(*writePageFP)(BL_Index, flashPage);


			// verify as we go //
			for(copyIndex = 0; copyIndex < FLASH_IMAGE_PAGE_SIZE; copyIndex++){ flashPage[copyIndex] = 0;}

			do{
				reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status
			}while(reg & 0x01);

			(*readPageFP)(BL_Index, flashPage);			// for now we're just reading
			fault = 0;
			for(copyIndex = 0; copyIndex < FLASH_IMAGE_PAGE_SIZE; copyIndex++){
				if(flashPage[copyIndex] != FW_SwapSpace[copyIndex + BL_Index]){
					fault++;
				}
			}
		}while(fault != 0);
	}

	do{
		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status
	}while(reg & 0x01);

	// read back.


	SWSPI_INITfp(0, FALSE);		// use the INIT function to switch over to the primary flash... but this time we don't need to reset the IC.
	do{
		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);				// Read Status
	}while(reg & 0x01);


	for(BL_Index = 0; BL_Index < flashImageSize; BL_Index = BL_Index + FLASH_IMAGE_PAGE_SIZE){
		(*readPageFP)((BL_Index), &(FW_SwapSpace[BL_Index]) );
	}


	crc16 = CRC_INITIAL;
	crc16 = CRCCalcRange(&(FW_SwapSpace[0]), flashImageSize);
	if(crc16 == flashImageCSUM16){
		fault = 0;
	}else{
		crc16 = CRC_INITIAL;
		crc16 = CRCCalcRange(&(FW_SwapSpace[FLASH_ADDR__IMAGE_1_START]), (flashImageSize - FLASH_ADDR__IMAGE_1_START));
		if(crc16 == flashImageCSUM16_imageOnly){
			fault = 0;
		}else{
			fault = 1;
		}
	}


	if(fault == 0){
		SWSPI_INITfp(1, FALSE);		// use the INIT function to switch over to the secondary flash... but this time we don't need to reset the IC.
		// dirty the image at 'flashTransferSource'.
		eraseSectorFP = &eraseSector;
		if(flashTransferSource == FLASH_ADDR__IMAGE_1_START){
			(*eraseSectorFP)(FLASH_ADDR__CFG_IMAGE_1);
		}else{
			(*eraseSectorFP)(FLASH_ADDR__CFG_IMAGE_2);
		}
		FW_SwapSpace[0 + (FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START)] = 0xFF;
	}
	// reset.
	do{		reg = SWSPI_ReadByte(FLASH_CMD_RDSR);	}while(reg & 0x01);  				// Read Status
	SWSPI_SendCmd(FLASH_CMD_RSTEN);						// RESET enable.
	SWSPI_SendCmd(FLASH_CMD_RST);						// RESET.
	SWSPI_INITfp(0, FALSE);		// use the INIT function to switch over to the primayr flash... but this time we don't need to reset the IC.
	SWSPI_SendCmd(FLASH_CMD_RSTEN);						// RESET enable.
	SWSPI_SendCmd(FLASH_CMD_RST);						// RESET.
	for(copyIndex = 0; copyIndex < 4096; copyIndex++){RESET_U8;}
	for(copyIndex = 0; copyIndex < 4096; copyIndex++){unRESET_U8;}

	ABORT_BOOTLOAD;
	NVIC_SystemReset();	// system reset.
}

__RAMFUNC(SRAM_ITC_cm7_BL) void transferBLandReboot(uint8_t *FW_SwapSpace)
{
	uint32_t reg, delay;
	uint32_t lastAddress;
	uint32_t index;
	// ram functions..
	void (*erase64KBlockFP)(uint32_t addr);
	void (*eraseSectorFP)(uint32_t addr);
	void (*SWSPI_INITfp)(uint32_t, uint32_t);
	void (*readPageFP)(uint32_t addr, uint8_t *data);
	void (*writePageFP)(uint32_t addr, uint8_t *data);

	erase64KBlockFP = &erase64KBlock;
	eraseSectorFP = &eraseSector;
	SWSPI_INITfp = &SWSPI_INIT;
	readPageFP = &readPage;
	writePageFP = &writePage;

	uint32_t	imageSize;
    uint32_t regPrimask;
    uint32_t    imageCRC;
	uint16_t packetCRC = CRC_INITIAL;


    regPrimask = DisableGlobalIRQ();

    // INIT flash.  With a Flash Reset!
	SWSPI_INIT(1, TRUE);

	// erase the image config page.
	WAIT_FOR_FLASH_READY;
	(*eraseSectorFP)(FLASH_ADDR__CFG_IMAGE_1);

	imageSize = ((uint32_t *)(&(FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START)])))[1];// = request->subtype2.imageSize;
	imageCRC =  ((uint32_t *)(&(FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START)])))[2];// = request->subtype2.crcChecksum;

	for(index = 0; index < imageSize; index = index + FLASH_IMAGE_64K_BLOCK_SIZE){

		WAIT_FOR_FLASH_READY;
		(*erase64KBlockFP)(FLASH_ADDR__IMAGE_1_START + index);
	}
	WDOG_Refresh(WDOG1);
	WAIT_FOR_FLASH_READY;
	PRINTF("FW Write: Erase Complete...\r\n");
//
	for(index = 0; index < imageSize; index = index + FLASH_PAGE_SIZE){
		WAIT_FOR_FLASH_READY;
		(*writePageFP)(FLASH_ADDR__IMAGE_1_START + index, &(FW_SwapSpace[index]));			// max write size is 1 page at a time (256 bytes)
	}
	WDOG_Refresh(WDOG1);
	WAIT_FOR_FLASH_READY;

	PRINTF("FW Write: Write to Flash Complete...\r\n");
	for(index = 0; index < imageSize; index = index + FLASH_PAGE_SIZE){
		(*readPageFP)(FLASH_ADDR__IMAGE_1_START + index, &(FW_SwapSpace[index]));
	}
	WDOG_Refresh(WDOG1);
	packetCRC = CRC_INITIAL;
	packetCRC = CRCCalcRange(&(FW_SwapSpace[0]), imageSize);
	PRINTF("FW Write: FLASH CRC: %x\r\n", packetCRC);

	if(packetCRC == imageCRC){
		// write image info.

		memset(flashPage, 0, sizeof(flashPage));
		((uint32_t *)(&flashPage))[0] = FLASH_CTRL__PartialEntityCode;
		((uint32_t *)(&flashPage))[1] = imageSize;
		((uint32_t *)(&flashPage))[2] = imageCRC;

		uint16_t packetCRC = CRC_INITIAL;
		packetCRC = CRCCalcRange(&(FW_SwapSpace[FLASH_ADDR__IMAGE_1_START]), (imageSize & 0x0FFFFFFF) - FLASH_ADDR__IMAGE_1_START);
		((uint32_t *)(&flashPage))[3] = packetCRC;

		WAIT_FOR_FLASH_READY;
		writePage(FLASH_ADDR__CFG_IMAGE_1, &(flashPage[0]));

		EnableGlobalIRQ(regPrimask);
	}
	NVIC_SystemReset();	// system reset.


}

