/*
 * app_storage.c
 */

#include <string.h>
#include <math.h>

#include "app_audio.h"
#include "fsl_pdm.h"
#include "fsl_pdm_edma.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_gpt.h"
#include "fsl_anatop_ai.h"

#include "app_shared.h"
#include "app_storage.h"


/* lwIP */
#include "lwip/opt.h"
#include "lwip/netifapi.h"
#include "lwip/tcpip.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/timeouts.h"
#include "lwip/tcp.h"
#include "lwip/sockets.h"

#include "board.h"
#include "app_microwave.h"
#include "IMD200x.h"
#include "app_jack.h"
#include "app_board.h"
#include "app_shared.h"
#include "app_PIR.h"
#include "app_accel.h"
#include "app_microwave.h"
#include "app_logging.h"
#include "senstarCRC.h"
#include "simpleFlashSupport.h"
#include "task.h"




// this returns processor stats that were pulled from the flash.
int32_t	loadStats(struct sys_stat_s *stats);
int32_t	saveStats(struct sys_stat_s *newStats);

union U_FlashStorageEntitySysStats storageEntitySysStats, storageEntitySysStatsCopy;

int32_t loadStats(struct sys_stat_s *stats)
{

	uint32_t i;


	union U_FlashStorageEntitySysStats storageEntity;
	void (*fp)(uint32_t, uint32_t);
	/* Program data buffer should be 4-bytes alignment, which can avoid busfault due to this memory region is configured as
	   Device Memory by MPU. */

	uint32_t	reg;

//	SWSPI_INIT();
	fp = &SWSPI_INIT;
	(*fp)(1,TRUE);

	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){
		readPage(FLASH_ADDR__SYS_STATS + (i * 256), &(storageEntity.raw[i][0]));
	}

	if(storageEntity.storageEntityCtrlData == FLASH_CTRL__ValidEntityCode)
	{
		PRINTF("Valid Stats in main Flash area.  Loading...\r\n");
		memcpy(&stats, &storageEntity.stats, sizeof(struct sys_stat_s));
		return 0;
	}
	PRINTF("Invalid stats in main flash area\r\n");
	// current settings are not valid.  check the backup settings
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){
		readPage(FLASH_ADDR__BACKUP_SYS_STATS + (i * 256), &(storageEntity.raw[i][0]));
	}

	if(storageEntity.storageEntityCtrlData == FLASH_CTRL__ValidEntityCode){
		// settings are valid.  load those.
		PRINTF("Backup stats are valid.  Using that.\r\n");
		defaultConfigFlag = 0;
		memcpy(&stats, &storageEntity.stats, sizeof(struct sys_stat_s));
	} else {
		// nope, going with just the defaults
		PRINTF("Backup stats are not Valid.  Using Defaults.\r\n");

		//PRINTF("SEAN... YOU NEED DEFALUTS... ADD DEFAULT STATS HERE.\r\n");
//		memset(&storageEntity, 0xFF, sizeof(storageEntity));
//		memcpy(&storageEntity.configData, &configParam, sizeof(configParam));
		stats->pwrstat.fault6V5_count = 0;
		stats->pwrstat.fault3V3_count = 0;
		stats->pwrstat.faultSideAPower_count = 0;
		stats->pwrstat.faultSideBPower_count = 0;
		stats->nwkstat.faultSideANetwork_count = 0;
		stats->nwkstat.faultSideBNetwork_count = 0;
		stats->sensorstat.faultAccel_count = 0;
		stats->sensorstat.faultLPIR_count = 0;
		stats->sensorstat.faultRPIR_count = 0;
		stats->sensorstat.faultRadar_count = 0;
		stats->sensorstat.faultHFMEMS_count = 0;
		stats->sensorstat.faultCamera_count = 0;
	}

	PRINTF("Saving to Main Flash area.\r\n");

	// if we're here, we need to update our main settings.
	eraseSector(FLASH_ADDR__SYS_STATS);
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){

		WAIT_FOR_FLASH_READY;

		writePage(FLASH_ADDR__SYS_STATS + (i * 256), &(storageEntity.raw[i][0]));
	}

	WAIT_FOR_FLASH_READY;

	// read back.
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){
		readPage(FLASH_ADDR__SYS_STATS + (i * 256), &(storageEntity.raw[i][0]));
	}
	// compare
	for(i = 0; i < sizeof(struct sys_stat_s); i++){
		if(((uint8_t *)(&storageEntity.stats))[i] != ((uint8_t *)(stats))[i]){

			PRINTF("Saving Failed.\r\n");
			break;	// mismatch
		}
	}
	if(i == sizeof(struct sys_stat_s)){
		SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
		SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__SYS_STATS, 0x00A0FFFF);			// overwrite the ctrl data to signify that this is good shit.
		PRINTF("Saving success!!.\r\n");
		return 0;
	}else{
		return -1; //failed
	}
}





int32_t	saveStats(struct sys_stat_s *newStats){
	if(sizeof(struct sys_stat_s) >= 255){
		return -1;
	}


	uint32_t mainValid = 0, backupValid = 0;
	uint8_t reg;


	/* Program data buffer should be 4-bytes alignment, which can avoid busfault due to this memory region is configured as
	   Device Memory by MPU. */
	uint32_t i;


	// check if maindata is valid.
	readPage(FLASH_ADDR__SYS_STATS, &(storageEntitySysStats.raw[0][0]));
	if((storageEntitySysStats.raw[0][0] == FLASH_CTRL__ValidEntityCode_0) &&
			(storageEntitySysStats.raw[0][1] == FLASH_CTRL__ValidEntityCode_1) &&
			(storageEntitySysStats.raw[0][2] == FLASH_CTRL__ValidEntityCode_2) &&
			(storageEntitySysStats.raw[0][3] == FLASH_CTRL__ValidEntityCode_3)) {
		mainValid = TRUE;
	}
	// check if backup data is valid.
	readPage(FLASH_ADDR__BACKUP_SYS_STATS, &(storageEntitySysStats.raw[0][0]));
	if((storageEntitySysStats.raw[0][0] == FLASH_CTRL__ValidEntityCode_0) &&
			(storageEntitySysStats.raw[0][1] == FLASH_CTRL__ValidEntityCode_1) &&
			(storageEntitySysStats.raw[0][2] == FLASH_CTRL__ValidEntityCode_2) &&
			(storageEntitySysStats.raw[0][3] == FLASH_CTRL__ValidEntityCode_3)) {
		backupValid = TRUE;
	}

	// if backup isn't valid... make the copy.
	// if backup doesn't match main... make the copy.  TODO
	if((backupValid == FALSE) && (mainValid == TRUE)){
		for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){
			readPage(FLASH_ADDR__SYS_STATS + (i * 256), &(storageEntitySysStats.raw[i][0]));
		}
		PRINTF("Invalid Backup Copy.  Saving Stats To backup area\r\n");

		storageEntitySysStats.storageEntityCtrlData = FLASH_CTRL__EmptyEntityCode;
		memcpy(&storageEntitySysStatsCopy, &storageEntitySysStats, sizeof(storageEntitySysStatsCopy));

		eraseSector(FLASH_ADDR__BACKUP_SYS_STATS);
		for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){

			WAIT_FOR_FLASH_READY;

			writePage(FLASH_ADDR__BACKUP_SYS_STATS + (i * 256), &(storageEntitySysStats.raw[i][0]));
		}

		WAIT_FOR_FLASH_READY;

		// read back.
		for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){
			readPage(FLASH_ADDR__BACKUP_SYS_STATS + (i * 256), &(storageEntitySysStats.raw[i][0]));
		}
		// compare
		for(i = 0; i < sizeof(struct sys_stat_s); i++){
			if(((uint8_t *)(&storageEntitySysStats.stats))[i] != ((uint8_t *)(&storageEntitySysStatsCopy.stats))[i]){
				break;	// mismatch
			}
		}
		if(i == sizeof(struct sys_stat_s)){
			SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
//			SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__BACKUP_SYS_STATS, FLASH_CTRL__ValidEntityCode);			// overwrite the ctrl data to signify that this is good shit.
			SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__BACKUP_SYS_STATS, 0x00A0FFFF);			// overwrite the ctrl data to signify that this is good shit.
			PRINTF("Saving Stats To backup ok\r\n");
		}else{
			PRINTF("Saving Stats To backup area Failed\r\n");
			return -1;	// abandon ship here... the read back was invalid.
		}
	}
	WAIT_FOR_FLASH_READY;
	// erase settings in main area.
	eraseSector(FLASH_ADDR__SYS_STATS);

	// load settings in to the main area.
	memcpy(&storageEntitySysStats.stats, newStats, sizeof(storageEntitySysStats.stats) - (((30 * 40)/8) * 3 * 4));
	storageEntitySysStats.storageEntityCtrlData = FLASH_CTRL__EmptyEntityCode;
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){
		WAIT_FOR_FLASH_READY;

		writePage(FLASH_ADDR__SYS_STATS + (i * 256), &(storageEntitySysStats.raw[i][0]));
	}

	WAIT_FOR_FLASH_READY;


	// read back.
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){
		readPage(FLASH_ADDR__SYS_STATS + (i * 256), &(storageEntitySysStats.raw[i][0]));
	}
	// compare
	for(i = 0; i < sizeof(struct sys_stat_s) - (((30 * 40)/8) * 3 * 4); i++){
		if(((uint8_t *)(&storageEntitySysStats.stats))[i] != ((uint8_t *)(newStats))[i]){
			break;	// mismatch
		}
	}
	if(i == sizeof(struct sys_stat_s) - (((30 * 40)/8) * 3 * 4)){
		SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
		SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__SYS_STATS, 0x00A0FFFF);			// overwrite the ctrl data to signify that this is good shit.
		PRINTF("Saving Stats To main ok\r\n");
	}else{
		PRINTF("Saving Stats To main Flash area Failed\r\n");
		return -1;	// abandon ship here... the read back was invalid.
	}


	// erase backup settings.
	eraseSector(FLASH_ADDR__BACKUP_SYS_STATS);

	// load setting in to the backup area.
	memcpy(&storageEntitySysStats.stats, newStats, sizeof(storageEntitySysStats.stats) - (((30 * 40)/8) * 3 * 4));
	storageEntitySysStats.storageEntityCtrlData = FLASH_CTRL__EmptyEntityCode;
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){
		WAIT_FOR_FLASH_READY;
		writePage(FLASH_ADDR__BACKUP_SYS_STATS + (i * 256), &(storageEntitySysStats.raw[i][0]));
	}
	WAIT_FOR_FLASH_READY;

	// read back.
	for(i = 0; i < FLASH_PAGES_PER_STORAGE_ENTITY_SYS_STATS; i++){
		readPage(FLASH_ADDR__BACKUP_SYS_STATS + (i * 256), &(storageEntitySysStats.raw[i][0]));
	}
	// compare
	for(i = 0; i < sizeof(struct sys_stat_s) - (((30 * 40)/8) * 3 * 4); i++){
		if(((uint8_t *)(&storageEntitySysStats.stats))[i] != ((uint8_t *)(newStats))[i]){
			break;	// mismatch
		}
	}
	if(i == sizeof(struct sys_stat_s) - (((30 * 40)/8) * 3 * 4)){
		SWSPI_SendCmd(FLASH_CMD_WREN);						// write enable.
		SWSPI_Write32bitWord(FLASH_CMD_4PP, FLASH_ADDR__BACKUP_SYS_STATS, 0x00A0FFFF);			// overwrite the ctrl data to signify that this is good shit.
		PRINTF("Saving Stats To backup ok\r\n");
	}else{
		PRINTF("Saving Stats To backup Flash area Failed\r\n");
		return -1;	// abandon ship here... the read back was invalid.
	}


	PRINTF("done saving stats data\r\n");
	return 0;

}










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

#define FLASH_PAGE_SIZE				 (256)		// write size
#define FLASH_SECTOR_SIZE			 (4096)		// minimum erase size.
#define FLASH_PAGES_PER_SECTOR		 ((FLASH_SECTOR_SIZE) / (FLASH_PAGE_SIZE))

#define FLASH_PAGES_IN_LOGGING_SPACE 	160
#define FLASH_SECTORS_IN_LOGGING_SPACE	10
#define FLASH_FAULT_LOG_SIZE__BYTES		(FLASH_SECTORS_IN_LOGGING_SPACE * FLASH_SECTOR_SIZE)
#define FLASH_FAULT_LOG_ENTRY_SIZE__BYTES		(sizeof(struct logEntry))
#define FLASH_FAULT_LOG_ENTRIES_PER_PAGE		((FLASH_PAGE_SIZE) / (FLASH_FAULT_LOG_ENTRY_SIZE__BYTES))						//8
#define FLASH_FAULT_LOG_ENTRIES_PER_SECTOR		((FLASH_SECTOR_SIZE) / (FLASH_FAULT_LOG_ENTRY_SIZE__BYTES))						//128
#define FLASH_FAULT_LOG_MAX_ENTRIES				((FLASH_FAULT_LOG_SIZE__BYTES) / (FLASH_FAULT_LOG_ENTRY_SIZE__BYTES))			//	1280

#define FLASH_ADDR__FAULT_LOG			(6 * FLASH_SECTOR_SIZE)
#define FLASH_ADDR__FAULT_LOG_BACKUP	((FLASH_ADDR__FAULT_LOG) + (FLASH_FAULT_LOG_SIZE__BYTES))




union U_flashLoggingPage{
	uint8_t raw[FLASH_PAGE_SIZE];
	struct logEntry entries[FLASH_FAULT_LOG_ENTRIES_PER_PAGE];
}faultLogPage;

union U_flashLoggingSector{
	uint8_t raw[FLASH_SECTOR_SIZE];
	uint8_t rawPages[FLASH_PAGES_PER_SECTOR][FLASH_PAGE_SIZE];
	struct logEntry entries[FLASH_FAULT_LOG_ENTRIES_PER_SECTOR];
}faultLogSector;

uint16_t indexList[FLASH_FAULT_LOG_MAX_ENTRIES][2];

void loadIndexList(void){
	uint32_t pageIndex, index, isBackup;
	uint16_t csum16;
	uint16_t indexList[FLASH_FAULT_LOG_MAX_ENTRIES][2];

	for(isBackup = 0; isBackup <2; isBackup++){
		for(pageIndex = 0; pageIndex < FLASH_PAGES_IN_LOGGING_SPACE; pageIndex++){

			if(isBackup){
				readPage(FLASH_ADDR__FAULT_LOG_BACKUP 	+ pageIndex * FLASH_PAGE_SIZE, (uint8_t *)&faultLogPage.raw);
			}else{
				readPage(FLASH_ADDR__FAULT_LOG 			+ pageIndex * FLASH_PAGE_SIZE, (uint8_t *)&faultLogPage.raw);
			}

			for(index = 0; index < FLASH_PAGES_PER_SECTOR; index++ ){
				csum16 = CRCCalcRange((uint8_t *) &(faultLogPage.entries[index].index), sizeof(faultLogPage.entries[0]) - sizeof(faultLogPage.entries[0].CSUM16));
				if(csum16 == faultLogPage.entries[index].CSUM16){
						// entry is valid.
					indexList[index + (pageIndex * FLASH_FAULT_LOG_ENTRIES_PER_PAGE)][isBackup] = faultLogPage.entries[index].index;
				}
			}
		}
	}
}


void validateFlash(void){


	uint32_t pageIndex, index, isBackup;
//	uint16_t csum16;
	uint16_t indexList[FLASH_FAULT_LOG_MAX_ENTRIES][2];

	loadIndexList();

	// index List now contains a listing of all the valid indexes in each page.  We now go over them to make sure they match.

	for(index = 0; index < FLASH_FAULT_LOG_MAX_ENTRIES; index++ ){
		if(indexList[index][0] != indexList[index][1]){
				// here we have a mismatch.
			if(indexList[index][1] == 0xFF){			// backup was blank.
				isBackup = 0;			// this is the valid section
			}else if(indexList[index][0] == 0xFF){		// main section was blank.
				isBackup = 1;
			}else{
					// both sections have data which claim valid checksums.  So, default to main.
				isBackup = 0;
			}

			// read the entire sector of the place where we're going to erase.
			for(pageIndex = 0; pageIndex < FLASH_PAGES_PER_SECTOR; pageIndex++){
				if(isBackup){
					readPage(FLASH_ADDR__FAULT_LOG				 	+ (pageIndex * FLASH_PAGE_SIZE) + (((index / FLASH_FAULT_LOG_ENTRIES_PER_SECTOR) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE) , (uint8_t *) &(faultLogSector.rawPages[pageIndex]));
				}else{
					readPage(FLASH_ADDR__FAULT_LOG_BACKUP 			+ (pageIndex * FLASH_PAGE_SIZE) + (((index / FLASH_FAULT_LOG_ENTRIES_PER_SECTOR) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE) , (uint8_t *) &(faultLogSector.rawPages[pageIndex]));
				}
			}
			// read the page of the sector with the valid data.
			if(isBackup){
				readPage(FLASH_ADDR__FAULT_LOG_BACKUP	 + (index / FLASH_FAULT_LOG_ENTRIES_PER_SECTOR), (uint8_t *) &(faultLogPage.raw));
			}else{
				readPage(FLASH_ADDR__FAULT_LOG 			 + (index / FLASH_FAULT_LOG_ENTRIES_PER_SECTOR), (uint8_t *) &(faultLogPage.raw));
			}

			// modify the sector.
			memcpy(&faultLogSector.entries[index % FLASH_FAULT_LOG_ENTRIES_PER_SECTOR], &(faultLogPage.entries[index % FLASH_FAULT_LOG_ENTRIES_PER_PAGE]), sizeof(faultLogPage.entries[0]) );

			// erase the page
			if(isBackup){
				eraseSector(FLASH_ADDR__FAULT_LOG 			+ ((index / FLASH_FAULT_LOG_ENTRIES_PER_SECTOR) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE);
			} else {
				eraseSector(FLASH_ADDR__FAULT_LOG_BACKUP	+ ((index / FLASH_FAULT_LOG_ENTRIES_PER_SECTOR) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE);
			}
			do{
				 taskYIELD();
			}while(IS_FLASH_BUSY);

			// write the page
			for(pageIndex = 0; pageIndex < FLASH_PAGES_PER_SECTOR; pageIndex++){
				if(isBackup){
					writePage(FLASH_ADDR__FAULT_LOG				 	+ (pageIndex * FLASH_PAGE_SIZE) + (((index / FLASH_FAULT_LOG_ENTRIES_PER_SECTOR) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE) , (uint8_t *) &(faultLogSector.rawPages[pageIndex]));
				}else{
					writePage(FLASH_ADDR__FAULT_LOG_BACKUP 			+ (pageIndex * FLASH_PAGE_SIZE) + (((index / FLASH_FAULT_LOG_ENTRIES_PER_SECTOR) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE) , (uint8_t *) &(faultLogSector.rawPages[pageIndex]));
				}
				do{
					 taskYIELD();
				}while(IS_FLASH_BUSY);
			}
		}

	}

}





#define SIZE_OF_ERROR_QUEUE		8
struct logEntry errorCodeQueue[SIZE_OF_ERROR_QUEUE];
uint32_t errorCodeQueueStartIndex = 0;
uint32_t errorCodeQueueEndIndex = 0;




struct STR_loggingStorageSupport{
	uint32_t loggingStartIndex;
	uint32_t loggingEndIndex;
	uint32_t lastIndexValue;
}loggingStorageSupport;


#if(0)
#define MAX_LOGGING_INDEX 65000
void app_storage_task(void *pvParameters)
{


	ssize_t err = ERR_OK;
	uint32_t pageIndex, byteIndex, index, isBackup;
	uint16_t csum16;
	struct logEntry newEntry;
//	ssize_t transmitCount;



	TickType_t	wakeTimer;
 //   struct sockaddr_storage dest_addr;

	memset(&indexList, 0xFF, sizeof(indexList));


	// first validate our data.
	// note that this will also load our index list.
	validateFlash();

	// for now, we're going to re-load the index list.  But this time, we know that backup data is identical to main data.
	loadIndexList();

	// find the end.  This will be the largest index.
	loggingStorageSupport.loggingEndIndex	= 0;
	loggingStorageSupport.lastIndexValue = 0;
	for(index = 0; index < FLASH_FAULT_LOG_MAX_ENTRIES; index++){
		if(indexList[index] != 0xFFFF){
			if(indexList[index] > loggingStorageSupport.lastIndexValue){
				loggingStorageSupport.lastIndexValue = indexList[index];
				loggingStorageSupport.loggingIndexEnd = index;
			}
		}
	}

	// special case where the index overflowed back to 0.
	if(loggingStorageSupport.loggingIndexValue >= MAX_LOGGING_INDEX){
		index = ((loggingStorageSupport.loggingIndexEnd + 1) % FLASH_FAULT_LOG_MAX_ENTRIES);
		if (indexList[index] == 0) {
				// when we overflow, the next value should be zero.  If not, we're on the edge of overflowing and thus our pointers were previously correct.
			loggingStorageSupport.lastIndexValue = indexList[index];
			loggingStorageSupport.loggingIndexEnd = index;
			// now we need to find the biggest number that's below (MAX_LOGGING_INDEX - FLASH_FAULT_LOG_MAX_ENTRIES)
			for(index = 0; index < FLASH_FAULT_LOG_MAX_ENTRIES; index++){
				if(indexList[index] < (MAX_LOGGING_INDEX - FLASH_FAULT_LOG_MAX_ENTRIES)){
					if(indexList[index] > loggingStorageSupport.lastIndexValue){
						loggingStorageSupport.lastIndexValue = indexList[index];
						loggingStorageSupport.loggingIndexEnd = index;
					}
				}
			}
		}
	}

	// default the start to the next index.
	loggingStorageSupport.startIndexValue = (loggingStorageSupport.loggingIndexEnd + 1) % FLASH_FAULT_LOG_MAX_ENTRIES;

	//index until we find the first non-FFFF value.
	for(index = 0; index < FLASH_FAULT_LOG_MAX_ENTRIES; index++){
		if(indexList[(index + loggingStorageSupport.startIndexValue) % FLASH_FAULT_LOG_MAX_ENTRIES] != 0xFFFF){
			loggingStorageSupport.startIndexValue = index;
			break;
		}
	}

	// but if this fails to find anything.  start index is zero.
	if(index == FLASH_FAULT_LOG_MAX_ENTRIES){
		loggingStorageSupport.startIndexValue = 0;
	}


	wakeTimer = xTaskGetTickCount();
	do{
		xTaskDelayUntil( &wakeTimer, 	100 / portTICK_PERIOD_MS );

		if(errorCodeQueueEndIndex == errorCodeQueueStartIndex){
			// nothing to write.
			continue;
		}

		// semaphore!!
		errorCodeQueueStartIndex = ((errorCodeQueueStartIndex + 1 ) % SIZE_OF_ERROR_QUEUE);
		memcpy(&newEntry, &errorCodeQueue[errorCodeQueueStartIndex], sizeof(newEntry));
		memset(&errorCodeQueue[errorCodeQueueStartIndex], 0xFFFF, sizeof(newEntry));
		// semaphore!!

		newEntry.index = (loggingStorageSupport.lastIndexValue + 1) % MAX_LOGGING_INDEX;
		newEntry.CSUM16 = CRCCalcRange((uint8_t *) &(newEntry.index), sizeof(newEntry) - sizeof(newEntry.CSUM16));

		loggingStorageSupport.loggingIndexEnd++;
		if((loggingStorageSupport.loggingIndexEnd % FLASH_FAULT_LOG_ENTRIES_PER_SECTOR) == 0x00){
				// our next entry is on a sector boundary.  So we wipe this sector.
			eraseSector(FLASH_ADDR__FAULT_LOG 			+ ((loggingStorageSupport.loggingIndexEnd / FLASH_FAULT_LOG_ENTRIES_PER_SECTOR) ));
			do{
				 taskYIELD();
			}while(IS_FLASH_BUSY);
		}

		readPage(FLASH_ADDR__FAULT_LOG + loggingStorageSupport.loggingIndexEnd * FLASH_FAULT_LOG_ENTRY_SIZE__BYTES, &faultLogPage);

		memcpy(&faultLogPage.entries[loggingStorageSupport.loggingIndexEnd % FLASH_FAULT_LOG_ENTRIES_PER_PAGE], &newEntry, sizeof(newEntry));

		writePage(FLASH_ADDR__FAULT_LOG + loggingStorageSupport.loggingIndexEnd * FLASH_FAULT_LOG_ENTRY_SIZE__BYTES, &faultLogPage);
		do{
			 taskYIELD();
		}while(IS_FLASH_BUSY);



	}while(1);







}

#endif

//  This will cause the writing of an error code to the flash memory.
//  This will queue the msg for writing and writing will happen within the
//  storage task.  I.e. this function is non blocking.
//  Return 0 for nomal execution
//  Return -1 for failure to write to queue (i.e. queue full).
//  Modifiers are error code specific and may be null
int32_t	writeErrorCode( uint32_t errorCode, int32_t modifier0, int32_t modifier1, int32_t modifier2, int32_t modifier3){
	if(((errorCodeQueueEndIndex + 1 ) % SIZE_OF_ERROR_QUEUE) ==  errorCodeQueueStartIndex ){
		// queue is full.
		return -1;
	}
	errorCodeQueueEndIndex = ((errorCodeQueueEndIndex + 1 ) % SIZE_OF_ERROR_QUEUE);

	errorCodeQueue[errorCodeQueueEndIndex].index = 0xFFFF;	//	filled in later
	errorCodeQueue[errorCodeQueueEndIndex].CSUM16 = 0xFFFF;	//	filled in later
	errorCodeQueue[errorCodeQueueEndIndex].errorCode = errorCode;
	errorCodeQueue[errorCodeQueueEndIndex].Modifier0 = modifier0;
	errorCodeQueue[errorCodeQueueEndIndex].Modifier1 = modifier1;
	errorCodeQueue[errorCodeQueueEndIndex].Modifier2 = modifier2;
	errorCodeQueue[errorCodeQueueEndIndex].Modifier3 = modifier3;

	return 0;
}








