


#include "math.h"
#include "MMA8451.h"
#include "app_shared.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_freeRTOS.h"
#include "board.h"
#include "IMD200x.h"
#include "fsl_edma.h"
#include "fsl_lpuart_edma.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_LPUART          LPUART2
#define DEMO_LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()
#define DEMO_LPUART_IRQn     LPUART2_IRQn
#define IMD2002_BAUD_RATE	256000

struct endianSwap16{
	uint8_t endianSwap16_8bit[2];
	uint16_t bit16;
};


#define ENDIAN_SWAP_16(number)	((uint16_t) (((number & 0xFF) << 8) | ((number & 0xFF00) >> 8)))
#define ENDIAN_SWAP_32(number) 	((uint32_t) (((number & 0xFF) << 24) | ((number & 0xFF00) << 8) | ((number & 0xFF0000) >> 8) | ((number & 0xFF000000) >> 24)))



__BSS(BOARD_SDRAM) IMD2002_TargetList_t IMD2002TargetList;
__BSS(BOARD_SDRAM) IMD2000_TargetList_t IMD2000TargetList;



edma_handle_t	LPUART_EdmaHandle_RX0;
edma_handle_t	LPUART_EdmaHandle_TX0;



#define U_IMD2002_CMD_REPLY_LENGTH 	32
#pragma pack(1)
union U_IMD2002_CMD_REPLY{
	uint8_t raw[U_IMD2002_CMD_REPLY_LENGTH];
	struct STR_IMD2002_CMD_REPLY{
		uint8_t SD;	// message type, either 68 orA2
		uint8_t LE;	// length
		uint8_t LEr;	// length repeated
		uint8_t SDr;	// message type repeated.
		uint8_t DA;	// destination address
		uint8_t SA;	// source address.
		uint8_t FC;	// function command.
		union{
			uint8_t  payload_8bit[4];	// replied setting.
			uint16_t payload_16bit[2];
			uint32_t payload_32bit;
			float payload_32bitFloat;
		};
		uint8_t FCS;	// checksum;
		uint8_t ED ; 	// end.  = 0x16
	}message;
}imd2002CmdReply;


#pragma pack(1)

#define U_IMD2002_TARGET_REPLY_LENGTH 314
union U_IMD2002_TARGET_REPLY{
	uint8_t raw[U_IMD2002_TARGET_REPLY_LENGTH];
	struct STR_IMD2002_TARGET_REPLY{
		uint8_t SD;	// message type, A2
		uint8_t DA;	// destination address
		uint8_t SA;	// source address.
		uint8_t FC;	// function command.
		uint16_t NR;		// Number of Targets
		uint16_t TID;		// target list ID
		uint16_t reserved1;
		uint16_t reserved2;

		union{
			IMD2002_Target_t target;
			float payload_32bitFloat[5];
		}payload[IMD2002_MAX_TARGETS];
		uint8_t FCS;	// checksum;
		uint8_t ED ; 	// end.  = 0x16
	}message;
};//imd2002TargetReply;

#pragma pack(1)
#define U_IMD2000_TARGET_REPLY_LENGTH 334
union U_IMD2000_TARGET_REPLY{
	uint8_t raw[U_IMD2000_TARGET_REPLY_LENGTH];
	struct STR_IMD2000_TARGET_REPLY{
		uint8_t SD;	// message type, A2
		uint8_t DA;	// destination address
		uint8_t SA;	// source address.
		uint8_t FC;	// function command.
		uint16_t NR;		// Number of Targets
		uint16_t TID;		// target list ID
		uint16_t reserved1;
		uint16_t reserved2;

		union{
			IMD2000_Target_t target;
			float payload_32bitFloat[4];
		}payload[IMD2000_MAX_TARGETS];
		uint8_t FCS;	// checksum;
		uint8_t ED ; 	// end.  = 0x16
	}message;
};//imd2000TargetReply;

__BSS(BOARD_SDRAM) union U_IMD200x_TARGET_REPLY{

	union U_IMD2002_TARGET_REPLY	IMD2002;
	union U_IMD2000_TARGET_REPLY	IMD2000;


}imd200xTargetReply;
#pragma pack()

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
//static void uart_task(void *pvParameters);
int IMD2002_decodeTargetFrame(unsigned char *frame_array,IMD2002_TargetList_t *targetList);
int IMD2000_decodeTargetFrame(unsigned char *frame_array,IMD2000_TargetList_t *targetList);


/*******************************************************************************
 * Code
 ******************************************************************************/
uint32_t	acquisitionRunning = 0;



extern volatile bool rxBufferEmpty;
extern volatile bool txBufferFull;
extern volatile bool txOnGoing;
extern volatile bool rxOnGoing;
extern lpuart_edma_handle_t g_lpuartEdmaHandle;
#include "fsl_iomuxc.h"





//__BSS(BOARD_SDRAM) uint8_t background_buffer[1024];		// a single target list can be up to 314bytes.
__BSS(SRAM_DTC_cm7) uint8_t background_buffer[320];		// a single target list can be up to 314bytes.
__BSS(SRAM_DTC_cm7) uint8_t txOutBufferLPUART2[32];
//lpuart_rtos_handle_t rtosUartHandle_IMD200x;
//struct _lpuart_handle uartHandle;
__BSS(SRAM_DTC_cm7) lpuart_rtos_handle_t rtosUartHandle_IMD200x;
__BSS(SRAM_DTC_cm7) struct _lpuart_handle uartHandle;
__BSS(SRAM_DTC_cm7) lpuart_transfer_t sendXfer;
__BSS(SRAM_DTC_cm7) lpuart_transfer_t receiveXfer;


lpuart_rtos_config_t lpuart_config = {
    .baudrate    = IMD2002_BAUD_RATE,
    .parity      = kLPUART_ParityDisabled,
    .stopbits    = kLPUART_OneStopBit,
    .buffer      = background_buffer,
    .buffer_size = sizeof(background_buffer),
    .base = LPUART2
};
lpuart_config_t	lpuartConfig;


void LPUART_UserCallback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_LPUART_TxIdle == status)
    {
        txBufferFull = false;
        txOnGoing    = false;
    }

    if (kStatus_LPUART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing     = false;
    }
}

//#define WRITE_CMD_BUFFER_SIZE 256
//__BSS(BOARD_SDRAM) unsigned char writeCmdMessage[WRITE_CMD_BUFFER_SIZE];
void IMD2002_writeCmd_SubCmd(unsigned int command, unsigned int subCommand)
{		// write a command, with a sub command.

	int i,j, checksum;
    sendXfer.data        = txOutBufferLPUART2;

			i = 0;
			txOutBufferLPUART2[i++] = 0x68;
			txOutBufferLPUART2[i++] = 0x05;		// message length, ignoring the 68, and 2 lenght bytes.
			txOutBufferLPUART2[i++] = 0x05;
			txOutBufferLPUART2[i++] = 0x68;
			txOutBufferLPUART2[i++] = 0x64;	// destination (IMD2002 = 0x64
			txOutBufferLPUART2[i++] = 0x01;	// source	   (IMD2002 = 0x01
			txOutBufferLPUART2[i++] = command;
			txOutBufferLPUART2[i++] = (char) (subCommand / 256);
			txOutBufferLPUART2[i++] = (char) (subCommand & 0xFF);
			checksum = 0;
			for(j = 4; j <i; j++){
				checksum += txOutBufferLPUART2[j];
			}
			txOutBufferLPUART2[i++] = checksum;
			txOutBufferLPUART2[i++] = 0x16;
			sendXfer.dataSize = i;

//            LPUART_RTOS_Send(&rtosUartHandle_IMD200x, sendXfer.data, sendXfer.dataSize);
            LPUART_SendEDMA(LPUART2, &g_lpuartEdmaHandle, &sendXfer);
            do{
            	vTaskDelay(1 / portTICK_PERIOD_MS );
            }while(txOnGoing == TRUE);

}
void IMD2002_writeCmd(unsigned int command)
{		// write a command, with a sub command.
//	lpuart_transfer_t sendXfer;
	int i,j, checksum;
    sendXfer.data        = txOutBufferLPUART2;

			i = 0;
			txOutBufferLPUART2[i++] = 0x68;
			txOutBufferLPUART2[i++] = 0x03;		// message length, ignoring the 68, and 2 lenght bytes.
			txOutBufferLPUART2[i++] = 0x03;
			txOutBufferLPUART2[i++] = 0x68;
			txOutBufferLPUART2[i++] = 0x64;	// destination (IMD2002 = 0x64
			txOutBufferLPUART2[i++] = 0x01;	// source	   (IMD2002 = 0x01
			txOutBufferLPUART2[i++] = command;
			checksum = 0;
			for(j = 4; j <i; j++){
				checksum += txOutBufferLPUART2[j];
			}
			txOutBufferLPUART2[i++] = checksum;
			txOutBufferLPUART2[i++] = 0x16;
			sendXfer.dataSize = i;

            LPUART_SendEDMA(LPUART2, &g_lpuartEdmaHandle, &sendXfer);
            do{
            	vTaskDelay(1 / portTICK_PERIOD_MS );
            }while(txOnGoing == TRUE);

}

void IMD2002_writeCmd_Value(unsigned int command, unsigned int subCommand, uint8_t *data, int len)
{		// write a command, with a sub command.
	int i,j, checksum;
    sendXfer.data        = txOutBufferLPUART2;

			i = 0;
			txOutBufferLPUART2[i++] = 0x68;
			txOutBufferLPUART2[i++] = 0x05 + len;		// message length, ignoring the 68, and 2 lenght bytes.
			txOutBufferLPUART2[i++] = 0x05 + len;
			txOutBufferLPUART2[i++] = 0x68;
			txOutBufferLPUART2[i++] = 0x64;	// destination (IMD2002 = 0x64
			txOutBufferLPUART2[i++] = 0x01;	// source	   (IMD2002 = 0x01
			txOutBufferLPUART2[i++] = command;
			txOutBufferLPUART2[i++] = (char) (subCommand / 256);
			txOutBufferLPUART2[i++] = (char) (subCommand & 0xFF);
			checksum = 0;
			for(j = 0; j < len; j++){
				txOutBufferLPUART2[i++] = data[len - j - 1];			// endian swapping here!!!
			}

			for(j = 4; j <i; j++){
				checksum += txOutBufferLPUART2[j];
			}
			txOutBufferLPUART2[i++] = checksum;
			txOutBufferLPUART2[i++] = 0x16;
			sendXfer.dataSize = i;

//            LPUART_RTOS_Send(&rtosUartHandle_IMD200x, sendXfer.data, sendXfer.dataSize);
		   LPUART_SendEDMA(LPUART2, &g_lpuartEdmaHandle, &sendXfer);
			do{
            	vTaskDelay(1 / portTICK_PERIOD_MS );
            }while(txOnGoing == TRUE);

}

int IMD2002_readAndParse(void)
{
	int i;
	int status;
	size_t receivedCount;
	uint32_t checksum;

    memset(&(imd2002CmdReply.raw[0]), 0, sizeof(imd2002CmdReply));
	while(rxOnGoing == TRUE){
    	vTaskDelay(1 / portTICK_PERIOD_MS );
	};
	memcpy(&(imd2002CmdReply.raw[0]), &background_buffer, sizeof(imd2002CmdReply));
//   	status = LPUART_RTOS_Receive(&rtosUartHandle_IMD200x, &(imd2002CmdReply.raw[0]),  4, &receivedCount);
//	if(status != kStatus_Success){
//		return status;
//	}

	if((imd2002CmdReply.message.SD == 0x68) && (imd2002CmdReply.message.SDr == 0x68)){
		// variable length message.
		if(imd2002CmdReply.message.LE == imd2002CmdReply.message.LEr){
			// length byte is repeated.
			// receive the remaining bytes.
//		   	status = LPUART_RTOS_Receive(&rtosUartHandle_IMD200x, &(imd2002CmdReply.raw[4]),  imd2002CmdReply.message.LE + 2, &receivedCount);
//			if(status != kStatus_Success){
//				return status;
//			}

			if((imd2002CmdReply.message.SA == 0x64) && (imd2002CmdReply.message.DA == 0x01)){
				// source and destination is correct.
				checksum = 0;
				for(i = 4; i< imd2002CmdReply.message.LE + 4; i++){
					checksum += imd2002CmdReply.raw[i];
				}
				if((checksum & 0xFF) == imd2002CmdReply.message.FCS){
						// passed checksum.
					if((imd2002CmdReply.message.ED == 0x16)){
						// correct end character.
					}else{return -1;}
				}else{return -1;}

			}else{return -1;}

		}else{return -1;}

	}else{return -1;}


	switch(imd2002CmdReply.message.FC){
		case(0xD2):
			break;
		default:
			break;
	}

	return 0;

}

void flushUART(void){

#if(0)
	uint32_t receivedCount, i;
	uint8_t data[256];
//	uint32_t status;
	do{
//		receivedCount = 0;
//		status = LPUART_TransferGetReceiveCount(rtosUartHandle_IMD200x.base, &uartHandle, &receivedCount);
		receivedCount = LPUART_TransferGetRxRingBufferLength(rtosUartHandle_IMD200x.base, &uartHandle);
		if(receivedCount == 0){ break;}
		if(receivedCount > sizeof(data)){receivedCount = sizeof(data);}
//		status = LPUART_RTOS_Receive(&rtosUartHandle_IMD200x, data,  receivedCount, &i);
		LPUART_RTOS_Receive(&rtosUartHandle_IMD200x, data,  receivedCount, &i);
		// TODO deal with RTOS_Receive returning failed.
	}while(receivedCount > 0);
#endif
}



__BSS(BOARD_SDRAM) struct IMD200x_settings IMD200xSettings ;
#pragma pack()


int IMD2002_writeCmd_parseReply(unsigned int command, unsigned int subCommand)
{		// write a command, with a sub command.

	uint32_t checksum;
	int i,j;
	int attemptCount;
    sendXfer.data        = txOutBufferLPUART2;

    		// this reset and LPUART INIT stuff is what's required if we hard aborted our previous transfer.  Not sure how much of this
    		// is actually necessary beyond the LPUART INIT command... I know that one is needed.  And maybe it can be paired down.. but that's a big TODO.
    EDMA_ResetChannel(LPUART_EdmaHandle_RX0.base, LPUART_EdmaHandle_RX0.channel);
    EDMA_ResetChannel(LPUART_EdmaHandle_TX0.base, LPUART_EdmaHandle_TX0.channel);
    LPUART_TransferCreateHandleEDMA(LPUART2, &g_lpuartEdmaHandle, LPUART_UserCallback, NULL,  &LPUART_EdmaHandle_TX0, &LPUART_EdmaHandle_RX0);
    LPUART_Init(LPUART2, &lpuartConfig, BOARD_DebugConsoleSrcFreq());				//  this is messed up... but if we did abort a DMA transfer, then the UART needs to be reset.  Not sure why.

			i = 0;
			txOutBufferLPUART2[i++] = 0x68;
			txOutBufferLPUART2[i++] = 0x05;		// message length, ignoring the 68, and 2 lenght bytes.
			txOutBufferLPUART2[i++] = 0x05;
			txOutBufferLPUART2[i++] = 0x68;
			txOutBufferLPUART2[i++] = 0x64;	// destination (IMD2002 = 0x64
			txOutBufferLPUART2[i++] = 0x01;	// source	   (IMD2002 = 0x01
			txOutBufferLPUART2[i++] = command;
			txOutBufferLPUART2[i++] = (char) (subCommand / 256);
			txOutBufferLPUART2[i++] = (char) (subCommand & 0xFF);
			checksum = 0;
			for(j = 4; j <i; j++){
				checksum += txOutBufferLPUART2[j];
			}
			txOutBufferLPUART2[i++] = checksum;
			txOutBufferLPUART2[i++] = 0x16;
			sendXfer.dataSize = i;

//            LPUART_RTOS_Send(&rtosUartHandle_IMD200x, sendXfer.data, sendXfer.dataSize);
			memset(background_buffer,0,sizeof(background_buffer));
		    receiveXfer.data = background_buffer;
		    receiveXfer.dataSize = sizeof(background_buffer);				// this lenght is longer than the longest message.  i.e. set to the size of our buffer.
		    if ((!rxOnGoing) && rxBufferEmpty)
		    {
		        rxOnGoing = true;
		        LPUART_ReceiveEDMA(LPUART2, &g_lpuartEdmaHandle, &receiveXfer);
		    }
			LPUART_SendEDMA(LPUART2, &g_lpuartEdmaHandle, &sendXfer);



	memset(&(imd2002CmdReply.raw[0]), 0, sizeof(imd2002CmdReply));
	vTaskDelay(20 / portTICK_PERIOD_MS );

	attemptCount = 0;
	while(rxOnGoing && attemptCount < 100){
		attemptCount++;
		vTaskDelay(5 / portTICK_PERIOD_MS );
		if(!((background_buffer[0] == 0x68) && (background_buffer[3] == 0x68) && (background_buffer[1] == background_buffer[2]))){
			continue;
		}
		checksum = 0;
		for(i = 4; i < (4 + background_buffer[2]); i++){
			checksum += background_buffer[i];
		}
		if(((checksum & 0xFF) == background_buffer[i]) && (background_buffer[i+1] == 0x16))  {
			break;
		}
	};
	LPUART_TransferAbortReceiveEDMA(LPUART2, &g_lpuartEdmaHandle);			// abort the transfer.
	rxOnGoing = FALSE;
	if(attemptCount >= 100){
		return -1;
	}
	memcpy(&imd2002CmdReply, background_buffer, sizeof(imd2002CmdReply));


	if((imd2002CmdReply.message.SD == 0x68) && (imd2002CmdReply.message.SDr == 0x68)){
		// variable length message.
		if(imd2002CmdReply.message.LE == imd2002CmdReply.message.LEr){
			// length byte is repeated.
			// receive the remaining bytes.

			if((imd2002CmdReply.message.SA == 0x64) && (imd2002CmdReply.message.DA == 0x01)){
				// source and destination is correct.
				checksum = 0;
				for(i = 4; i< imd2002CmdReply.message.LE + 4; i++){
					checksum += imd2002CmdReply.raw[i];
				}
				if(imd2002CmdReply.message.LE == 5){	// ugh... sooo dirty... sooo dirty.
					imd2002CmdReply.message.FCS = imd2002CmdReply.message.payload_8bit[2];
					imd2002CmdReply.message.ED = imd2002CmdReply.message.payload_8bit[3];
				}else if(imd2002CmdReply.message.LE == 3){		// more dirtyness...
					imd2002CmdReply.message.FCS = imd2002CmdReply.message.payload_8bit[0];
					imd2002CmdReply.message.ED = imd2002CmdReply.message.payload_8bit[1];
				}
				if((checksum & 0xFF) == imd2002CmdReply.message.FCS){
						// passed checksum.
					if((imd2002CmdReply.message.ED == 0x16)){
						// correct end character.
					}else{return -1;}
				}else{return -1;}

			}else{return -1;}

		}else{return -1;}

	}else{return -1;}


	switch(imd2002CmdReply.message.FC){
		case(0xD1):
			// start / stop acquisition
			// There's a common reply from either start or stop acquisition... kinda dumb.
			if(subCommand == 0x00){
				acquisitionRunning = 1;
			}else
			{
				acquisitionRunning = 0;
			} break;
		case(0xD2):
			if(subCommand == 0x00){
				// serial number.
				IMD200xSettings.serialNumber =  ENDIAN_SWAP_32(imd2002CmdReply.message.payload_32bit);
			}

		break;
		case(0xD4):
			if(subCommand == 0x00){
				IMD200xSettings.velocity.raw32[0] = ENDIAN_SWAP_32(imd2002CmdReply.message.payload_32bit);
			}else if(subCommand == 0x01){
				IMD200xSettings.velocity.raw32[1] = ENDIAN_SWAP_32(imd2002CmdReply.message.payload_32bit);
			}else if(subCommand == 0x02){
				IMD200xSettings.range.raw32[0] = ENDIAN_SWAP_32(imd2002CmdReply.message.payload_32bit);
			}else if(subCommand == 0x03){
				IMD200xSettings.range.raw32[1] = ENDIAN_SWAP_32(imd2002CmdReply.message.payload_32bit);
			}else if(subCommand == 0x04){
				IMD200xSettings.signal.raw32[0] = ENDIAN_SWAP_32(imd2002CmdReply.message.payload_32bit);
			}else if(subCommand == 0x05){
				IMD200xSettings.signal.raw32[1] = ENDIAN_SWAP_32(imd2002CmdReply.message.payload_32bit);
			}else if(subCommand == 0x07){
				IMD200xSettings.falseAlarmSuppression = ENDIAN_SWAP_16(imd2002CmdReply.message.payload_16bit[0]);
			}else if(subCommand == 0x08){
				IMD200xSettings.freqChannel = ENDIAN_SWAP_16(imd2002CmdReply.message.payload_16bit[0]);
			}


		break;
		case(0xD6):
				if(subCommand == 0x0104){
					// product info.  determines if we're a 2000 or a 2002.
					IMD200xSettings.productInfo = ENDIAN_SWAP_16(imd2002CmdReply.message.payload_16bit[0]);
				}
		break;
		default:
		break;
	}
	return 0;
}



uint8_t IMD2002_init( struct IMD200x_settings *newSettings ){

	int validSettings = 0;
	int attempts = 10;

	lpuart_config.srcclk = BOARD_DebugConsoleSrcFreq();
    NVIC_SetPriority(LPUART2_IRQn, 5);



#if(0)


    if (kStatus_Success != LPUART_RTOS_Init(&rtosUartHandle_IMD200x, &uartHandle, &lpuart_config))
    {
        vTaskSuspend(NULL);
    }

#else
//    lpuart_transfer_t xfer;

    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = IMD2002_BAUD_RATE;
    lpuartConfig.enableTx     = true;
    lpuartConfig.enableRx     = true;

    LPUART_Init(LPUART2, &lpuartConfig, BOARD_DebugConsoleSrcFreq());
#if(0)
do{
    xfer.data     = txOutBufferLPUART2;
    xfer.dataSize = 10;//sizeof(txOutBufferLPUART2) - 1;
    txOnGoing     = true;
    txOutBufferLPUART2[0] = 1;
    txOutBufferLPUART2[1] = 2;
    txOutBufferLPUART2[2] = 3;
    txOutBufferLPUART2[3] = 4;
    txOutBufferLPUART2[4] = 5;
    txOutBufferLPUART2[5] = 6;
    txOutBufferLPUART2[6] = 7;
    txOutBufferLPUART2[7] = 8;
    txOutBufferLPUART2[8] = 9;
    txOutBufferLPUART2[9] = 10;

    LPUART_SendEDMA(LPUART2, &g_lpuartEdmaHandle, &xfer);
    vTaskDelay(500 / portTICK_PERIOD_MS );
    vTaskDelay(500 / portTICK_PERIOD_MS );

    sendXfer.data = txOutBufferLPUART2;
    sendXfer.dataSize = 11;

    txOutBufferLPUART2[0] = 0x68;
    txOutBufferLPUART2[1] = 0x05;
    txOutBufferLPUART2[2] = 0x05;
    txOutBufferLPUART2[3] = 0x68;
    txOutBufferLPUART2[4] = 0x64;
    txOutBufferLPUART2[5] = 0x01;
    txOutBufferLPUART2[6] = 0xD2;
    txOutBufferLPUART2[7] = 0x00;
    txOutBufferLPUART2[8] = 0x00;
    txOutBufferLPUART2[9] = 0x37;
    txOutBufferLPUART2[10] = 0x16;

    receiveXfer.data = background_buffer;
    receiveXfer.dataSize = 13;
    if ((!rxOnGoing) && rxBufferEmpty)
    {
        rxOnGoing = true;
        LPUART_ReceiveEDMA(LPUART2, &g_lpuartEdmaHandle, &receiveXfer);
    }
    LPUART_SendEDMA(LPUART2, &g_lpuartEdmaHandle, &sendXfer);
    vTaskDelay(500 / portTICK_PERIOD_MS );
    vTaskDelay(500 / portTICK_PERIOD_MS );


}while(1);
#endif

#endif



    do{


		memset(&IMD200xSettings, 0, sizeof(IMD200xSettings));
		flushUART();
		stopAcquisition();	// stop

		IMD2002_writeCmd_Value(0xD5, 0x00, &(newSettings->velocity.raw8[0]), 4);
		vTaskDelay(10 / portTICK_PERIOD_MS );
		IMD2002_writeCmd_Value(0xD5, 0x01, &(newSettings->velocity.raw8[4]), 4);
		vTaskDelay(10 / portTICK_PERIOD_MS );
		IMD2002_writeCmd_Value(0xD5, 0x02, &(newSettings->range.raw8[0]), 4);
		vTaskDelay(10 / portTICK_PERIOD_MS );
		IMD2002_writeCmd_Value(0xD5, 0x03, &(newSettings->range.raw8[4]), 4);
		vTaskDelay(10 / portTICK_PERIOD_MS );
		IMD2002_writeCmd_Value(0xD5, 0x04, &(newSettings->signal.raw8[0]), 4);
		vTaskDelay(10 / portTICK_PERIOD_MS );
		IMD2002_writeCmd_Value(0xD5, 0x05, &(newSettings->signal.raw8[4]), 4);
		vTaskDelay(10 / portTICK_PERIOD_MS );

		IMD2002_writeCmd_Value(0xD5, 0x07, (uint8_t *) &(newSettings->falseAlarmSuppression), 2);
		vTaskDelay(10 / portTICK_PERIOD_MS );
		IMD2002_writeCmd_Value(0xD5, 0x08, (uint8_t *) &(newSettings->freqChannel), 2);
		vTaskDelay(10 / portTICK_PERIOD_MS );
		flushUART();
	//    IMD2002_writeCmd_SubCmd(0xD2, 0x00);
	//	vTaskDelay(20 / portTICK_PERIOD_MS );   //McuWait_WaitOSms(1);
	//    IMD2002_readAndParse();
		IMD2002_writeCmd_parseReply(0xD2, 0x00);
		IMD2002_writeCmd_parseReply(0xD4, 0x00);
		IMD2002_writeCmd_parseReply(0xD4, 0x01);
		IMD2002_writeCmd_parseReply(0xD4, 0x02);
		IMD2002_writeCmd_parseReply(0xD4, 0x03);
		IMD2002_writeCmd_parseReply(0xD4, 0x04);
		IMD2002_writeCmd_parseReply(0xD4, 0x05);
		IMD2002_writeCmd_parseReply(0xD4, 0x07);
		IMD2002_writeCmd_parseReply(0xD4, 0x08);
		IMD2002_writeCmd_parseReply(0xD6, 0x104);
		validSettings = 0;

		validSettings |= IMD200xSettings.velocity.lower != newSettings->velocity.lower;
		validSettings |= IMD200xSettings.velocity.upper != newSettings->velocity.upper;
		validSettings |= IMD200xSettings.range.lower != newSettings->range.lower;
		validSettings |= IMD200xSettings.range.upper != newSettings->range.upper;
		validSettings |= IMD200xSettings.signal.lower != newSettings->signal.lower;
		validSettings |= IMD200xSettings.signal.upper != newSettings->signal.upper;
		validSettings |= IMD200xSettings.freqChannel != newSettings->freqChannel;
		validSettings |= IMD200xSettings.falseAlarmSuppression != newSettings->falseAlarmSuppression;
		validSettings |= !((IMD200xSettings.productInfo == 2002) || (IMD200xSettings.productInfo == 2000));
		attempts--;
    }while((validSettings != 0) && (attempts > 0));

    if(attempts > 0){
    	startAcquisition();	// start
    	return 0;
    }else{
    	return -1;
    }

}

void stopAcquisition(void){

	IMD2002_writeCmd_parseReply(0xD1, 0x01);	// 1 = stop
}

void startAcquisition(void){

	IMD2002_writeCmd_parseReply(0xD1, 0x00);	// 0 = start
}


void requestNewTargetList(void){


	LPUART_TransferAbortReceiveEDMA(LPUART2, &g_lpuartEdmaHandle);			// abort any previous transfer.
    rxOnGoing = false;														// reset the flags.
    rxBufferEmpty = true;
    memset(background_buffer, 0 , sizeof(background_buffer));
    receiveXfer.data = background_buffer;
    receiveXfer.dataSize = sizeof(imd200xTargetReply.IMD2002.raw);			// setup the transfer structure.

	EDMA_ResetChannel(LPUART_EdmaHandle_RX0.base, LPUART_EdmaHandle_RX0.channel);	//	we need to reset the DMA.  In case the previous transfer was hard reset.
    EDMA_ResetChannel(LPUART_EdmaHandle_TX0.base, LPUART_EdmaHandle_TX0.channel);
    LPUART_TransferCreateHandleEDMA(LPUART2, &g_lpuartEdmaHandle, LPUART_UserCallback, NULL,  &LPUART_EdmaHandle_TX0, &LPUART_EdmaHandle_RX0);
    LPUART_Init(LPUART2, &lpuartConfig, BOARD_DebugConsoleSrcFreq());				//  this is messed up... but if we did abort a DMA transfer, then the UART needs to be reset.  Not sure why.
    if ((!rxOnGoing) && rxBufferEmpty)
    {
        rxOnGoing = true;
        LPUART_ReceiveEDMA(LPUART2, &g_lpuartEdmaHandle, &receiveXfer);				// start the receive.
    }

	IMD2002_writeCmd(0xDA);													// send the command.

}


uint32_t receiveNewTargetList(void){

	int i;
	uint32_t checksum;

    memset(&(imd200xTargetReply.IMD2002.raw[0]), 0, sizeof(imd200xTargetReply.IMD2002));

    if(rxOnGoing){
    	return -1;
    }
	if(background_buffer[0] != 0xA2){
		return -1;
	}
    memcpy(&(imd200xTargetReply.IMD2002.raw[0]), &background_buffer, sizeof(imd200xTargetReply.IMD2002));

	if((imd200xTargetReply.IMD2002.message.SD == 0xA2)){
		// variable length message.
		imd200xTargetReply.IMD2002.message.NR = ENDIAN_SWAP_16(imd200xTargetReply.IMD2002.message.NR);
		if(imd200xTargetReply.IMD2002.message.NR <= IMD2002_MAX_TARGETS){
			// length byte is repeated.
			// receive the remaining bytes.
			if((imd200xTargetReply.IMD2002.message.SA == 0x64) && (imd200xTargetReply.IMD2002.message.DA == 0x01)){
				// source and destination is correct.
				checksum = 0;
				for(i = 1; i< sizeof(imd200xTargetReply.IMD2002.raw) - 2; i++){
					checksum += imd200xTargetReply.IMD2002.raw[i];
				}
				if((checksum & 0xFF) == imd200xTargetReply.IMD2002.message.FCS){
						// passed checksum.
					if((imd200xTargetReply.IMD2002.message.ED == 0x16)){
						// correct end character.
					}else{
						return -1;
					}
				}else{
					return -1;
				}
			}else{
				return -1;
			}
		}else{
			return -1;
		}
	}else{
		return -1;
	}

	memset(&IMD2002TargetList, 0, sizeof(IMD2002TargetList));
	IMD2002_decodeTargetFrame(imd200xTargetReply.IMD2002.raw, &IMD2002TargetList);


	return 0;
}




int IMD2002_decodeTargetFrame(unsigned char *frame_array, IMD2002_TargetList_t *targetList){
	uint16_t ui16_fc;
	uint16_t ui16_nrOfTargets;
	uint8_t *pData;
	uint32_t ui32_tmp;
//	uint16_t ui16_tmp;
	uint16_t ui16_targetCounter;

	if(frame_array[0] == 0x68) /* check SD2 Frame */
	{
		return IMD2002_API_ERR_COMMAND_NO_VALID_FRAME_FOUND;
	/* variable length frames not supported */
	} else {
		ui16_fc = 3; /* set function code bit for fixed length frames */
//	} ui16_nrOfTargets = ((uint16_t)(frame_array[ui16_fc+1] & 0x00ff) << 8) + (uint16_t)(frame_array[ui16_fc+2] & 0x00ff);
	} ui16_nrOfTargets = ((uint16_t)(frame_array[ui16_fc+2] & 0x00ff) << 8) + (uint16_t)(frame_array[ui16_fc+1] & 0x00ff);

	if(ui16_nrOfTargets > IMD2002_MAX_TARGETS){
		return IMD2002_API_ERR_COMMAND_FAILURE;
	}
	targetList->ui16_nrOfTargets = ui16_nrOfTargets;
	targetList->ui16_targetListId = ((uint16_t)(frame_array[ui16_fc+3] & 0x00ff) << 8) + (uint16_t)(frame_array[ui16_fc+4] & 0x00ff);
	targetList->ui16_reserved1 = ((uint16_t)(frame_array[ui16_fc+5] & 0x00ff) << 8) + (uint16_t)(frame_array[ui16_fc+6] & 0x00ff);
	targetList->ui16_reserved2 = ((uint16_t)(frame_array[ui16_fc+7] & 0x00ff) << 8) + (uint16_t)(frame_array[ui16_fc+8] & 0x00ff);
	pData = &frame_array[ui16_fc+9];
	for(ui16_targetCounter = 0u; ui16_targetCounter < ui16_nrOfTargets; ui16_targetCounter++){
		ui32_tmp = (((*pData++)&0x000000ff) << 24);
		ui32_tmp|=(((*pData++)&0x000000ff) << 16);
		ui32_tmp|=(((*pData++)&0x000000ff) << 8);
		ui32_tmp|=((*pData++)&0x000000ff);
		targetList->target[ui16_targetCounter].f32_range_m = *(float *)&ui32_tmp; ui32_tmp = (((*pData++)&0x000000ff) << 24);
		ui32_tmp|=(((*pData++)&0x000000ff) << 16);
		ui32_tmp|=(((*pData++)&0x000000ff) << 8);
		ui32_tmp|=((*pData++)&0x000000ff);
		targetList->target[ui16_targetCounter].f32_velocity_mps = *(float *)&ui32_tmp; ui32_tmp = (((*pData++)&0x000000ff) << 24);
		ui32_tmp|=(((*pData++)&0x000000ff) << 16);
		ui32_tmp|=(((*pData++)&0x000000ff) << 8);
		ui32_tmp|=((*pData++)&0x000000ff);
		targetList->target[ui16_targetCounter].f32_signal_dB = *(float *)&ui32_tmp;
		ui32_tmp = (((*pData++)&0x000000ff) << 24);
		ui32_tmp|=(((*pData++)&0x000000ff) << 16);
		ui32_tmp|=(((*pData++)&0x000000ff) << 8);
		ui32_tmp|=((*pData++)&0x000000ff);
		targetList->target[ui16_targetCounter].f32_estimatedTimeOfArrival_s = *(float *)&ui32_tmp;
		ui32_tmp = (((*pData++)&0x000000ff) << 24);
		ui32_tmp|=(((*pData++)&0x000000ff) << 16);
		ui32_tmp|=(((*pData++)&0x000000ff) << 8);
		ui32_tmp|=((*pData++)&0x000000ff);
		targetList->target[ui16_targetCounter].f32_incidentAngle_deg = *(float *)&ui32_tmp;
	}
	return IMD2002_API_ERR_OK;
}


#define FIELD_X_RANGE 			(5*2)		// this is the max distance from the origin in the X direction.  i.e. +/-5m in both directions (it's 5x2 because it's half meters scaled).
#define FIELD_Y_RANGE 			(10*2)		// this is the max distance from the origin in the Y direction.

#define POLAR_TO_CARTESIAN_X(r, angle)	((r) * cos((angle) * 3.1415926535 / 180))
#define POLAR_TO_CARTESIAN_Y(r, angle)  ((r) * sin((angle) * 3.1415926535 / 180))
#define POLAR_TO_CARTESIAN_X_Y(radius, angle, x,y) {x = POLAR_TO_CARTESIAN_X(radius, angle);  y = POLAR_TO_CARTESIAN_Y(radius, angle);}


#define timRound(x) (((fmod(x,1)) > 0.5) ? ( ((int)x) + 1 ) : ( ((int)x)))

void IMD2000wrapper(float distance, float weight)
{

	int angle;
	float xCoord=0, yCoord=0, prevX, prevY;



	if(distance > FIELD_Y_RANGE) return;
	if(distance <= 0) return;
	if(weight <= 0) return;


	prevX = 0; prevY = 0;
//	memset(new_IMD2000Canvas_targetWeights_XY,0,sizeof(new_IMD2000Canvas_targetWeights_XY));

		// support for IMD2000.  i.e. no angle component.
		for(angle = 0; angle < 180; angle++){

			POLAR_TO_CARTESIAN_X_Y(distance, angle, xCoord, yCoord);			// (distance, angle, x , y)
			if((xCoord > 0) && ( xCoord <0.5)){ xCoord = 0.5;}	// near field equates to lowest bin and doesn't get rounded down.
			xCoord = timRound(2 * xCoord);	// units are in number of half meters.
			yCoord = timRound(2 * yCoord);	// units are in number of half meters.
			if((xCoord >= FIELD_X_RANGE) || (xCoord < (0 - FIELD_X_RANGE))) {continue;}
			if((yCoord >= FIELD_Y_RANGE) || (yCoord < 0)) {continue;}
			if((xCoord == prevX) && (yCoord == prevY)){ continue; }
			prevX = xCoord;	prevY = yCoord;






	}
}


