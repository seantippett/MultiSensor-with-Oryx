/*
 * app_board.c
 *
 *  Created on: Jun. 28, 2022
 *      Author: tsnider
 */


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
#include "fsl_lpspi.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "app_config.h"
#include "phy_ksz9563.h"


volatile bool gptIsrFlag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/



int writeMsg_withTimeout_socketSendTo(const int sock, uint8_t *dataptr, size_t size, struct sockaddr *clientAddr );
int writeMsg_withTimeout_socket(const int sock, uint8_t *dataptr, size_t size);


void loadDefaultSettings(void);
void loadFlashSettings(void);

void nodeEntryMaintenance(void);


void getHHMMSS(uint8_t *hh, uint8_t *mm, uint8_t *ss);


#include "fsl_iomuxc.h"
#if(0)
void app_board_task(void *pvParameters)
{
//	int32_t pir1data, pir2data;
//	uint32_t pirError = NO_ERROR;
	TickType_t wakeTimer;
//	loadCurrentSettings();


	wakeTimer = xTaskGetTickCount();
	xTaskDelayUntil( &wakeTimer, 	250 / portTICK_PERIOD_MS );
	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
	PRINTF("Board task: Startup. \r\n");
	xSemaphoreGive( xPrintMutex );


   	/* Main loop.   */
   	while (1)
   	{
   		wakeTimer = xTaskGetTickCount();
   		xTaskDelayUntil( &wakeTimer, 	250 / portTICK_PERIOD_MS );



   	}
}
#endif
#define UDP_SYNC_PORT_NUMBER 15001
#define UDP_BUFFER_SIZE 512
uint8_t udpBuffer[UDP_BUFFER_SIZE];
uint32_t isMaster = 0;
uint32_t sendTimeSyncFlag = 0;
uint32_t displaySyncTimeFlag = 0;
uint32_t displaySyncTimeSentFlag = 0;

bool sync_flag = 0; // for the diagnostic msg
uint8_t sync_error_change = 0;


int32_t currentTimeIncrementationFactor = 0;
#define SIZE_OF_NODE_LIST		10		//MAX_NODE_ADDRESS
#if(1)

#pragma pack(1)
struct STR_NODE_ADDR_DATA{
	   uint32_t	ipAddr;			//uint8_t	ipAddr[4];		easier to treat as an uint32, rather than 4 uint8's .
	   uint32_t	nodeNumber;
	   uint32_t	uid;
	   uint64_t currentTimestamp;
}nodeList[SIZE_OF_NODE_LIST];
uint32_t nodeListExpiration[SIZE_OF_NODE_LIST];
#define NODE_LIST_AGING_COUNTER_RESET_VALUE 		1000000
#pragma pack(0)


#define UDP_PACKET_ID__TIME_PACKET		0x00000001
#pragma pack(1)
struct STR_UDP_TIMEPACKET{
		uint32_t		STX;
		uint32_t		packetId;
		uint32_t		packetSize;
		uint32_t		nodeNumber;
		struct 	STR_NODE_ADDR_DATA nodeList[SIZE_OF_NODE_LIST];
		uint32_t		crc16;
}udpTimePacket;
#pragma pack(0)

void sendTimeSync(void){
	sendTimeSyncFlag = 1;
}

void updateNodeEntries( struct STR_NODE_ADDR_DATA (*newEntries)[SIZE_OF_NODE_LIST]){
	uint32_t i,j;


	for(i = 0; i < SIZE_OF_NODE_LIST; i++){
		if(newEntries[i]->ipAddr ==  0){
			break;	// no more entries.
		}
		for(j = 0; j < SIZE_OF_NODE_LIST; j++){
			if(nodeList[j].ipAddr ==  0){
				// no more entries.
				// add this entry to our node list.
				memcpy(&(nodeList[j].ipAddr), &(newEntries[i]->ipAddr), sizeof(nodeList[j].ipAddr));
				memcpy(&(nodeList[j].nodeNumber), &(newEntries[i]->nodeNumber), sizeof(nodeList[j].nodeNumber));
				memcpy(&(nodeList[j].uid), &(newEntries[i]->uid), sizeof(nodeList[j].uid));
				nodeListExpiration[i] = NODE_LIST_AGING_COUNTER_RESET_VALUE;
				break;
			}
			if(nodeList[j].ipAddr == newEntries[i]->ipAddr){
				nodeListExpiration[i] = NODE_LIST_AGING_COUNTER_RESET_VALUE;
				break;
			}
		}
	}
}

#if(0)
//https://github.com/MichaelDipperstein/sockets/blob/master/echoserver_udp.c
extern uint32_t uid[2];
void app_board_task(void *pvParameters)
{


	ssize_t err = NO_ERROR;
	uint32_t i;
//	ssize_t transmitCount;



	TickType_t	wakeTimer;
 //   struct sockaddr_storage dest_addr;

    int socketHandle;
	uint16_t packetCRC;

//	struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
	 struct sockaddr_in clientAddr;
//    struct sockaddr_storage source_addr;

    socklen_t addrLen;

    struct sockaddr_in sockaddr;
    memset(&clientAddr, 0, sizeof(clientAddr));

	/* Main loop. Get sensor data and send via TCP */
	while (1)
	{

		// create socket descriptor
		socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_TCP);
		if(socketHandle < 0){
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("UDPSync - Unable to create socket: errno %d\r\n", errno);
			xSemaphoreGive( xPrintMutex );
			break; //goto CLEAN_UP;
		}

		// allow Internet data from any address on our port

		memset(&sockaddr,0,sizeof(sockaddr));
		sockaddr.sin_family = AF_INET	;					//internet addr family
		sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);		// any incomming address
		sockaddr.sin_port = htons(UDP_SYNC_PORT_NUMBER);	// port number.

		// bind socket to local address.
		err = bind(socketHandle, (struct sockaddr *)&sockaddr, sizeof(sockaddr));

		if(err < 0){
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("UDPSync - Unable to bind socket: errno %d\r\n", err);
			xSemaphoreGive( xPrintMutex );
			break; //goto CLEAN_UP;

		}
		 // enable send / RX timeout.
	    struct timeval sendTimeout, recvTimeout;
	    sendTimeout.tv_sec = 2;
	    sendTimeout.tv_usec = 0;		// 2s
	    recvTimeout.tv_sec = 0;
	    recvTimeout.tv_usec = 1000;		//0.001 S
		//{
	    //  long    tv_sec;         /* seconds */
	    //  long    tv_usec;        /* and microseconds */
	    //};
       err = setsockopt(socketHandle, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));	// enable send timeout.
       err = setsockopt(socketHandle, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));	// enable rx timeout.

		wakeTimer = xTaskGetTickCount();


		while(1){
			xTaskDelayUntil( &wakeTimer, 	100 / portTICK_PERIOD_MS );
			nodeEntryMaintenance();

			if(sendTimeSyncFlag){
#if(1)
				udpTimePacket.STX = STX_VAL;
				udpTimePacket.packetSize = SIZE_OF_NODE_LIST * sizeof(nodeList[0]) + sizeof(udpTimePacket) - sizeof(nodeList);
				udpTimePacket.packetId = UDP_PACKET_ID__TIME_PACKET;

				memcpy(udpTimePacket.nodeList, &nodeList, sizeof(nodeList));
				udpTimePacket.nodeNumber = configParam.nodeAddress;
				udpTimePacket.nodeList[udpTimePacket.nodeNumber].currentTimestamp = currentTime_uS;
				udpTimePacket.nodeList[udpTimePacket.nodeNumber].ipAddr = *((uint32_t *)(&configParam.ipAddress));
				udpTimePacket.nodeList[udpTimePacket.nodeNumber].nodeNumber = configParam.nodeAddress;
				udpTimePacket.nodeList[udpTimePacket.nodeNumber].uid = uid[1];

				packetCRC = CRC_INITIAL;
				for(i = 0; i < sizeof(udpTimePacket) - sizeof(udpTimePacket.crc16); i++){
					CRCCalc(((uint8_t *)(&udpTimePacket))[i], &packetCRC);
				}
				udpTimePacket.crc16 = packetCRC;
#define IPADDR_LOCAL_BROADCAST    ((u32_t)0xC0A800ffUL)

				clientAddr.sin_family = AF_INET	;					//internet addr family
				clientAddr.sin_addr.s_addr = htonl(IPADDR_LOCAL_BROADCAST);
				clientAddr.sin_port = htons(UDP_SYNC_PORT_NUMBER);	// port number.
				writeMsg_withTimeout_socketSendTo(socketHandle, (uint8_t *)&udpTimePacket, sizeof(udpTimePacket), (struct sockaddr *)&clientAddr);
				clientAddr.sin_addr.s_addr = htonl(INADDR_ANY);		// any incomming address
				sendTimeSyncFlag = FALSE;
				displaySyncTimeSentFlag = 1;
				currentTimeIncrementationFactor = 0;
#endif
			}


				/* use recvfrom so we can know where the data came from */
			err = recvfrom(socketHandle, udpBuffer, UDP_BUFFER_SIZE, 0,	(struct sockaddr *)&clientAddr, &addrLen);

			if (err < 0){
				continue;
			}else{
                /* we received a valid message */
                char from[INET_ADDRSTRLEN + 1];
                from[0] = '\0';

                if (NULL !=
                    inet_ntop(AF_INET, (void *)&(clientAddr.sin_addr), from,
                        INET_ADDRSTRLEN))
                {
//                	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//                	printf("Received message from %s:%d: \n", from, ntohs(clientAddr.sin_port));
//                	xSemaphoreGive( xPrintMutex );
                }
                else
                {
        			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
                    printf("Received message from unresolveble address\n");
        			xSemaphoreGive( xPrintMutex );
        			continue;
                }


				struct STR_UDP_TIMEPACKET *packetPtr = ((struct STR_UDP_TIMEPACKET *)udpBuffer);	//convenience pointer
                if((((struct STR_UDP_TIMEPACKET *)udpBuffer)->STX) != STX_VAL){
                	continue;
                }

				packetCRC = CRC_INITIAL;
				for(i = 0; i < sizeof(udpTimePacket) - sizeof(udpTimePacket.crc16); i++){
					CRCCalc(((uint8_t *)(&udpBuffer))[i], &packetCRC);
				}
				if(packetPtr->crc16 != packetCRC){
					continue;
				}

                // valid msg.
                if((packetPtr->packetId) == UDP_PACKET_ID__TIME_PACKET){

                	if((packetPtr->nodeNumber < configParam.nodeAddress)){
                		// time sync packet is from lower node address, so take this as higher significance.

                		if( (currentTime_uS > (packetPtr->nodeList[packetPtr->nodeNumber].currentTimestamp + 1000000)) ||
                				(currentTime_uS < (packetPtr->nodeList[packetPtr->nodeNumber].currentTimestamp - 1000000)))
                		{
                			//	more than 1s out.
                			// just adopt the new current time.
                			currentTime_uS = packetPtr->nodeList[packetPtr->nodeNumber].currentTimestamp;
                			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
                            printf("Greater than 1s time correction\n");
                			xSemaphoreGive( xPrintMutex );

                		}
                		else if(currentTime_uS > (packetPtr->nodeList[packetPtr->nodeNumber].currentTimestamp)){
                			currentTimeIncrementationFactor -= 500;
                		}
                		else if(currentTime_uS < (packetPtr->nodeList[packetPtr->nodeNumber].currentTimestamp)){
                			currentTimeIncrementationFactor += 500;
                		}
                		displaySyncTimeFlag = 1;
                		sync_flag = 1;
                	}

                	updateNodeEntries( &(((struct STR_UDP_TIMEPACKET *)udpBuffer)->nodeList)  );


                }




			}



		}
	}


}
#endif


void nodeEntryMaintenance(void){
	uint32_t i,j;
	struct STR_NODE_ADDR_DATA swapSpace;

	// 'age' the node entries.
	for(i = 0; i < SIZE_OF_NODE_LIST; i++){
		if(nodeList[i].ipAddr ==  0){
			break;	// no more entries.
		}
		if(nodeListExpiration[i] > 0){
			nodeListExpiration[i]--;
		}
	}
	// let expired addresses 'sink'.
	for(i = 0; i < SIZE_OF_NODE_LIST - 1; i++){
		if(nodeList[i].ipAddr ==  0){
			break;	// no more entries.
		}
		for(j = i; j < SIZE_OF_NODE_LIST - 1; j++){
			if(nodeList[j].ipAddr ==  0){
				break;	// no more entries.
			}
			if(nodeListExpiration[i] ==  0){
				memcpy(&swapSpace, &(nodeList[j]), sizeof(swapSpace));
				memcpy(&(nodeList[j]), &(nodeList[j + 1]), sizeof(swapSpace));
				memcpy(&(nodeList[j+1]), &swapSpace, sizeof(swapSpace));
			}
		}
	}
//	nodeListItemCount = i;

}



#endif










