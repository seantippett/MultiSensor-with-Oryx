#ifndef __SIMPLE_FLASH_SUPPORT__H
#define __SIMPLE_FLASH_SUPPORT__H



#include <stdint.h>
#include "app_config.h"



#define FLASH_CMD_RDSR	0x05		// read status, table 6.1
#define FLASH_CMD_WRSR	0x01		// Write status, table 6.1

#define FLASH_CMD_EN4B 0xB7
#define FLASH_CMD_RDBR 0x16		// read bank register
#define FLASH_CMD_WRBRV 0x17	// write bank register

#define FLASH_CMD_NORD	0x03	// Normal Read
#define FLASH_CMD_WREN	0x06	// Write Enable
#define FLASH_CMD_WRDI	0x04	// Write Disable

#define FLASH_CMD_PP	0x02	// page program (256 Byte)

#define FLASH_CMD_SER	0xD7	// Sector Erase (4Kbyte)	- minimum erase size.


// Page Program Time	0.8mS	//	4Kbyte
// Byte Program Time	40uS	//  1 byte
// Sector Erase Time	300mS	//	4Kbyte
// Block Erase Time		0.5S	//	32Kbyte



#if(0)
	#define SWSPI_CLK_1		GPIO10->DR_SET = 0x002000
	#define SWSPI_CLK_0 	GPIO10->DR_CLEAR = 0x002000		//
	#define SWSPI_CSn_1		GPIO10->DR_SET = 0x004000		//
	#define SWSPI_CSn_0 	GPIO10->DR_CLEAR = 0x004000

	#define SWSPI_RESETn_1 	GPIO10->DR_SET = 0x00200
	#define SWSPI_RESETn_0 	GPIO10->DR_CLEAR = 0x00200		//
	#define SWSPI_WPn_1 	GPIO10->DR_SET = 0x400
	#define SWSPI_WPn_0 	GPIO10->DR_CLEAR = 0x400

	#define SWSPI_MOSI_1		GPIO10->DR_SET = 0x001000	//
	#define SWSPI_MOSI_0 		GPIO10->DR_CLEAR = 0x001000
	#define SWSPI_MISO_BIT		0x00000800					// GPIO10, IO11, GPIO_SD_B2_01
#endif

#define HALF_BIT_DELAY	1//10
//#define PAGE_PROG_DELAY 65535


#define FLASH_PAGES_PER_STORAGE_ENTITY	5
#define FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS	1
#ifndef FLASH_PAGE_SIZE
	#define FLASH_PAGE_SIZE					(256)
#endif

#define FLASH_CTRL__PartialEntityCode	0xFFFFA0A0
#define FLASH_CTRL__ValidEntityCode		0xFFFFA000
#define FLASH_CTRL__CorruptedDataCode	0x00FFA0A0
#define FLASH_CTRL__EmptyEntityCode		0xFFFFFFFF

#define FLASH_CTRL__ValidEntityCode_0	0x00
#define FLASH_CTRL__ValidEntityCode_1	0xA0
#define FLASH_CTRL__ValidEntityCode_2	0xFF
#define FLASH_CTRL__ValidEntityCode_3	0xFF

#define FLASH_CTRL__PartialEntityCode_0	0xA0
#define FLASH_CTRL__PartialEntityCode_1	0xA0
#define FLASH_CTRL__PartialEntityCode_2	0xFF
#define FLASH_CTRL__PartialEntityCode_3	0xFF



#define FLASH_PORT_U8				0
#define FLASH_PORT_U9				1


#define FLASH_ADDR__SETTINGS		0
#define FLASH_ADDR__BACKUP_SETTINGS	4096		// one sector over.

#define FLASH_ADDR__SYS_STATS			 8192
#define FLASH_ADDR__BACKUP_SYS_STATS	12288		// one sector over.




#define FLASH_ADDR__IMAGE_1_START	0x00100000
#define FLASH_ADDR__IMAGE_1_END   	0x01FFFFFF

#define FLASH_ADDR__IMAGE_2_START	0x02100000
#define FLASH_ADDR__IMAGE_2_END   	0x03FFFFFF

#define FLASH_IMAGE_SECTOR_COUNT		8192			// sectors are minimum erase granularity
#define FLASH_IMAGE_SECTOR_SIZE			4096

#define FLASH_IMAGE_64K_BLOCK_COUNT		512				// 64k blocks are biggest erase granularity
#define FLASH_IMAGE_64K_BLOCK_SIZE		(0x10000)


#define FLASH_IMAGE_PAGE_COUNT			(FLASH_IMAGE_SECTOR_COUNT * 16)		// pages are minimum erase granularity
#define FLASH_IMAGE_PAGE_SIZE			(256)//(FLASH_IMAGE_SECTOR_SIZE / 16 )


#define FLASH_ADDR__CFG_IMAGE_1		0x000F000		// this tells us about the image
#define FLASH_ADDR__CFG_IMAGE_2		0x000FF000		// this tells us about the image


#define FLASH_CFG__IMAGE_CTRL_0		0x00
#define FLASH_CFG__IMAGE_CTRL_1		0x01
#define FLASH_CFG__IMAGE_CTRL_2		0x02
#define FLASH_CFG__IMAGE_CTRL_3		0x03

#define FLASH_CFG__IMAGE_SIZE_0		0x04
#define FLASH_CFG__IMAGE_SIZE_1		0x05
#define FLASH_CFG__IMAGE_SIZE_2		0x06
#define FLASH_CFG__IMAGE_SIZE_3		0x07

#define FLASH_CFG__IMAGE_CSUM_0		0x08
#define FLASH_CFG__IMAGE_CSUM_1		0x09
#define FLASH_CFG__IMAGE_CSUM_2		0x0A
#define FLASH_CFG__IMAGE_CSUM_3		0x0B


#define FLASH_MAIN_CODE_SPACE_START 0x00100000	//	this must be on a block boundary (64k block size).  otherwise boot load erase will mess up.





#define FLASH_CMD_RDSR	0x05		// read status, table 6.1
#define FLASH_CMD_WRSR	0x01		// Write status, table 6.1

#define FLASH_CMD_EN4B 0xB7
#define FLASH_CMD_RDBR 0x16		// read bank register
#define FLASH_CMD_WRBRV 0x17	// write bank register

#define FLASH_CMD_NORD	0x03	// Normal Read - 3 byte or 4 byte addressing based on a status bit.
#define FLASH_CMD_4NORD	0x13	// 4 byte Normal Read - 4 byte addressing.
#define FLASH_CMD_WREN	0x06	// Write Enable
#define FLASH_CMD_WRDI	0x04	// Write Disable

#define FLASH_CMD_PP	0x02	// page program (256 Byte) - 3 byte or 4 byte addressing based on a status bit.
#define FLASH_CMD_4PP	0x12	// page program (256 Byte) - 4 byte addressing.

#define FLASH_CMD_4SER	0x21	// Sector Erase (4Kbyte)	- minimum erase size. - 4 byte addressing.
#define FLASH_CMD_BER64	0xDC	// Sector Erase (64Kbyte)	- max erase size. - 4 byte addressing.

#define FLASH_CMD_RSTEN 0x66
#define FLASH_CMD_RST   0x99


// Page Program Time	0.8mS	//	4Kbyte
// Byte Program Time	40uS	//  1 byte
// Sector Erase Time	300mS	//	4Kbyte
// Block Erase Time		0.5S	//	32Kbyte


extern uint32_t	SWSPI_clk_Bit;
extern uint32_t	SWSPI_csn_Bit;
extern uint32_t	SWSPI_rst_Bit;
extern uint32_t	SWSPI_wpn_Bit;
extern uint32_t	SWSPI_mosi_Bit;
extern uint32_t	SWSPI_miso_Bit;


#define SWSPI_U9_CLK_BIT	0x00002000
#define SWSPI_U9_CSn_BIT	0x00004000
//#define SWSPI_U9_CSn_BIT	0x00000040	//-	 THIS REQUIRES A HW mod on the board.  Also a change in the init for swspi
#define SWSPI_U9_WPn_BIT	0x00000400
#define SWSPI_U9_RST_BIT	0x00000200
#define SWSPI_U9_MOSI_BIT	0x00001000
#define SWSPI_U9_MISO_BIT	0x00000800

#define SWSPI_U8_CLK_BIT	0x00010000
#define SWSPI_U8_CSn_BIT	0x00008000
#define SWSPI_U8_WPn_BIT	0x00080000
#define SWSPI_U8_RST_BIT	0x00100000
#define SWSPI_U8_MOSI_BIT	0x00020000
#define SWSPI_U8_MISO_BIT	0x00040000


#define SET_SWSPI_FOR_PRIMARY	SWSPI_clk_Bit = SWSPI_U8_CLK_BIT; \
							SWSPI_csn_Bit = SWSPI_U8_CSn_BIT; \
							SWSPI_rst_Bit = SWSPI_U8_WPn_BIT; \
							SWSPI_wpn_Bit = SWSPI_U8_RST_BIT; \
							SWSPI_mosi_Bit = SWSPI_U8_MOSI_BIT; \
							SWSPI_miso_Bit = SWSPI_U8_MISO_BIT;

#define SET_SWSPI_FOR_SECONDARY	SWSPI_clk_Bit = SWSPI_U9_CLK_BIT; \
							SWSPI_csn_Bit = SWSPI_U9_CSn_BIT; \
							SWSPI_rst_Bit = SWSPI_U9_WPn_BIT; \
							SWSPI_wpn_Bit = SWSPI_U9_RST_BIT; \
							SWSPI_mosi_Bit = SWSPI_U9_MOSI_BIT; \
							SWSPI_miso_Bit = SWSPI_U9_MISO_BIT;

#define ABORT_BOOTLOAD		*((uint32_t *)0x400E81B4) = 0x05; *((uint32_t *)0x400E81B8) = 0x05; *((uint32_t *)0x400E81BC) = 0x05; *((uint32_t *)0x400E81C0) = 0x05; *((uint32_t *)0x400E81C4) = 0x05;	*((uint32_t *)0x400E81C8) = 0x11; *((uint32_t *)0x400E81CC) = 0x01; *((uint32_t *)0x400E81D0) = 0x11; *((uint32_t *)0x400E81D4) = 0x01; *((uint32_t *)0x400E81D8) = 0x01; *((uint32_t *)0x400E81DC) = 0x01;	*((uint32_t *)0x400E81E0) = 0x11;


							// MUX controllers.





#define SWSPI_CLK_1		GPIO10->DR_SET = SWSPI_clk_Bit
#define SWSPI_CLK_0 	GPIO10->DR_CLEAR = SWSPI_clk_Bit		//
#define SWSPI_CSn_1		GPIO10->DR_SET = SWSPI_csn_Bit		//
#define SWSPI_CSn_0 	GPIO10->DR_CLEAR = SWSPI_csn_Bit

#define SWSPI_RESETn_1 	GPIO10->DR_SET = SWSPI_rst_Bit
#define SWSPI_RESETn_0 	GPIO10->DR_CLEAR = SWSPI_rst_Bit		//
#define SWSPI_WPn_1 	GPIO10->DR_SET = SWSPI_wpn_Bit
#define SWSPI_WPn_0 	GPIO10->DR_CLEAR = SWSPI_wpn_Bit

#define SWSPI_MOSI_1		GPIO10->DR_SET = SWSPI_mosi_Bit	//
#define SWSPI_MOSI_0 		GPIO10->DR_CLEAR = SWSPI_mosi_Bit
#define SWSPI_MISO_BIT		SWSPI_miso_Bit					//



#define RESET_U8		GPIO10->DR_CLEAR = SWSPI_U8_RST_BIT
#define unRESET_U8		GPIO10->DR_SET = SWSPI_U8_RST_BIT


struct processorStats{
	uint32_t	runTime_s;
	uint32_t	rebootCount;
	int16_t		minTemperature_k;
	int16_t		maxTemperature_k;
	uint32_t	tempMinTime_s;		// units are seconds after Jan 1, 1970.
	uint32_t	tempMaxTime_s;		// units are seconds after Jan 1, 1970.
	float		inputVoltsMin;			//
	float		inputVoltsMax;
	uint32_t	inputVoltsMinTime_s;		// units are seconds after Jan 1, 1970.
	uint32_t	inputVoltsMaxTime_s;		// units are seconds after Jan 1, 1970.
};



union U_FlashStorageEntity{
	uint8_t	raw[FLASH_PAGES_PER_STORAGE_ENTITY][FLASH_PAGE_SIZE];
	struct {
		uint32_t		storageEntityCtrlData;
		config_param_t	configData;
	};
};
union U_FlashStorageEntitySysStats{
	uint8_t	raw[FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS][FLASH_PAGE_SIZE];
	struct {
		uint32_t		storageEntityCtrlData;
		struct sys_stat_s	stats;
	};
};



// Read Status
// wait for flash writes to be marked as done.
#define WAIT_FOR_FLASH_READY					do{reg = SWSPI_ReadByte(FLASH_CMD_RDSR);}while(reg & 0x01);
#define IS_FLASH_BUSY							(SWSPI_ReadByte(FLASH_CMD_RDSR) & 0x01)
extern void eraseSector(uint32_t addr);
extern uint32_t SWSPI_ReadByte(uint8_t cmd);	// word length up to 4 bytes.
extern void loadFlashSettings(void);
extern void SWSPI_INIT(uint32_t port, uint32_t resetFlashChip);
extern void readPage(uint32_t addr, uint8_t *data);
extern void writePage(uint32_t addr, uint8_t *data);
extern void SWSPI_SendCmd(uint8_t cmd);
extern void SWSPI_Write32bitWord(uint8_t cmd, uint32_t address, uint32_t word);


// this returns processor stats that were pulled from the flash.
int32_t	getStatsFromLog(struct processorStats *stats);

struct logEntry {
	uint16_t	CSUM16;
	uint16_t	index;
	uint32_t	time;
	uint32_t	errorCode;
	uint32_t	Modifier0;
	uint32_t	Modifier1;
	uint32_t	Modifier2;
	uint32_t	Modifier3;
	uint32_t	reserved;
};

//  This will cause the writing of an error code to the flash memory.
//  This will queue the msg for writing and writing will happen within the
//  logging task.  I.e. this function is non blocking.
//  Return 0 for nomal execution
//  Return -1 for failure to write to queue (i.e. queue full).
//  Modifiers are error code specific and may be null
int32_t	writeErrorCode( uint32_t errorCode, int32_t modifier0, int32_t modifier1, int32_t modifier2, int32_t modifier3);

//Returns a value from 0 to 1024 representing number of error codes in the log.
int32_t	getNumbEventsInLog(void);


// entryNumber is 0 to 1023, representing an entry number that is desired.  Entries are in order of occurance
// *entry is a pointer to valid memory which will be populated with the error.
// returns 0 for nomal execution
// returns -1 if entry doesn't exist.
// Index / CSUM are for diagnostics use only.  Not for humans.
// Error Code / Modifiers are as per table
// Time is unix time (seconds after Jan 1st 1970)

int32_t	getErrorFromLog(uint32_t entryNumber, struct logEntry *entry);






#endif
