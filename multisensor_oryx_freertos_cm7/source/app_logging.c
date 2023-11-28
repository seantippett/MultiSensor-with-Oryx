/*
 * app_logging.c
 *
 *  Created on: July. 13, 2022
 *      Author: tsnider
 */


#include <string.h>
#include <math.h>

#include "cr_section_macros.h"

#include "app_audio.h"
#include "fsl_pdm.h"
#include "fsl_pdm_edma.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_gpt.h"
#include "fsl_anatop_ai.h"

#include "app_shared.h"
//#include "app_shield.h"

//#include "McuWait.h"
//#include "McuRTOS.h"



// ORYX
#include "core/net.h"

#include "board.h"
#include "app_microwave.h"
#include "IMD200x.h"
#include "app_jack.h"
#include "app_logging.h"
#include "app_video.h"
#include "JPEGENC.h"
#include "app_config.h"
#pragma pack(1)
struct STR_RadarQueue{
	int head;
	int tail;
	int size;
	struct STR_LOGGING_RadarElement elements[800];
};
__BSS(BOARD_SDRAM) struct STR_RadarQueue radarQueue, radarQueueCopy;
#pragma pack()

#pragma pack(1)
struct STR_CameraTargetQueue{
	int head;
	int tail;
	int size;
	struct STR_Jack_CameraTarget elements[1600];
};
__BSS(BOARD_SDRAM) struct STR_CameraTargetQueue cameraQueue, cameraQueueCopy;
#pragma pack()



#define PIR_QUEUE_SIZE	400
#pragma pack(1)
struct STR_PIRQueue{
	int head;
	int tail;
	int size;
	uint8_t elements[2][PIR_QUEUE_SIZE];
};
__BSS(BOARD_SDRAM) struct STR_PIRQueue pirQueue, pirQueueCopy;
#pragma pack()

#pragma pack(1)
//struct STR_AudioQueue{
//	uint32_t head;
//	uint32_t tail;
//	uint32_t size;
//	uint16_t elements[320000][3];
//};
__BSS(BOARD_SDRAM) struct STR_AudioQueue audioQueue, audioQueueCopy;
#pragma pack()

#pragma pack(1)
struct STR_AccelQueue{
	int head;
	int tail;
	int size;
	int16_t elements[3][16000];
};
__BSS(BOARD_SDRAM) struct STR_AccelQueue accelQueue, accelQueueCopy;
#pragma pack()

#pragma pack(1)
struct STR_CameraQueue{
	unsigned int head;
	unsigned int tail;
	unsigned int size;
	struct STR_LOGGING_VideoElement elements[20];
};
__BSS(BOARD_SDRAM) struct STR_CameraQueue videoQueue, videoQueueCopy;
#pragma pack()



#define TX_BUFFER_SIZE (1024 * 10)
#define RX_BUFFER_SIZE (1024 * 10)
struct S_txRxQueue{
	uint8_t *elements;
	uint32_t size;
	uint32_t head;
	uint32_t tail;
}txBuffer, rxBuffer;

uint32_t socketStatus;


int sendJack_2_3(void);
int handleReceive(uint8_t *buffer, int msgLenght);

SemaphoreHandle_t xLoggingMutex;

uint32_t enqueueTXRX(struct S_txRxQueue *queue, uint8_t *newElements, uint32_t size){
	uint32_t i = 0;
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
	for(i = 0; i < size; i++){
		if(queue->head < (queue->size -1)){
			// most cases
			if(queue->head + 1 == queue->tail){
				// queue is full.
				break;
			}
		}else{
			// queue head is at the end, look if tail is at 0.
			if(queue-> tail == 0){
				// queue is full.
				break;
			}
		}
		// we know queue has space.

		queue->elements[queue->head] = newElements[i];
		queue->head++;
		if(queue->head >= queue->size){queue->head = 0;}
	}
	xSemaphoreGive( xLoggingMutex);

	return i;	// i = number of items enqueued.
}

uint32_t getQueueLength(struct S_txRxQueue *queue){
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
	uint32_t returnValue = 0;
	if(queue->head == queue->tail){
		returnValue = 0;
	}
	else if(queue->head > queue->tail){
		returnValue = queue->head - queue->tail;
	}else	// queue->tail > queue->head
	{
		returnValue = queue->size - queue->tail + queue->head;
	}

	if((returnValue >= queue->size) || (returnValue < 0))
	{		// debug.  just checking.
		returnValue = 0;
	}


	xSemaphoreGive( xLoggingMutex);


	return returnValue;
}

// inspect does the same as getQueue, but it doesn't dequeue the elements.
uint32_t inspectQueueTXRX(struct S_txRxQueue *queue, uint8_t *elements, uint32_t size){
	uint32_t queueIndex;
	uint32_t queueEnd;
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
	queueEnd = queue->tail;
	for(queueIndex = 0; queueIndex < size; queueIndex++){
		if(queue->head == queueEnd){
			//queue is empty
			break;
		}
		elements[queueIndex] = queue->elements[queueEnd];
		queueEnd++;
		if(queueEnd >= queue->size) {
			queueEnd = 0;
		}
	}
	xSemaphoreGive( xLoggingMutex);

	return queueIndex;	// queueIndex = number of items enqueued.
}

uint32_t getQueueTXRX(struct S_txRxQueue *queue, uint8_t *elements, uint32_t size){
	uint32_t queueIndex;

	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
	for(queueIndex = 0; queueIndex < size; queueIndex++){
		if(queue->head == queue->tail){
			//queue is empty
			break;
		}
		elements[queueIndex] = queue->elements[queue->tail];
		queue->tail++;
		if(queue->tail >= queue->size) {
			queue->tail = 0;
		}
	}
	xSemaphoreGive( xLoggingMutex);

	return queueIndex;	// queueIndex = number of items enqueued.
}




int32_t addMessageToQueues_blocking(uint8_t *buffer, uint32_t size){

	uint32_t index, socketStatusCopy;

	index = 0;
	do{
		index += enqueueTXRX(&txBuffer, buffer + index, size - index);
		if(index == size) {break;}
		vTaskDelay(0);
		xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
			socketStatusCopy = socketStatus;
		xSemaphoreGive( xLoggingMutex);
		if(socketStatusCopy == 0){
			return -1;	// socket is closed.
		}
	}while(index < size);

	return 0;	// valid
}





void enqueueVideoElement(JPEGIMAGE *jpgImage, uint32_t imageSize, uint32_t numbElements){

	int i;

	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	for(i = 0; i < numbElements; i++){
		videoQueue.head++;
		if(videoQueue.head >= videoQueue.size){ videoQueue.head = 0;}
		if(videoQueue.head == videoQueue.tail){
			videoQueue.tail++;
			if(videoQueue.tail >= videoQueue.size){
				videoQueue.tail = 0;
			}
		}

		memcpy(&(videoQueue.elements[videoQueue.head].image), jpgImage->pOutput, sizeof(videoQueue.elements[videoQueue.head].image));
		videoQueue.elements[videoQueue.head].imageSize = imageSize;
	}
	xSemaphoreGive( xLoggingMutex);

}
// this returns the most recent video image and it's size (in bytes).
// note that you need to provide up to JPG_IMAGE_BUFFER_SIZE of space available at *jpg
void getVideoElement(uint32_t *jpgSize, uint8_t *jpg)
{
	uint32_t queueIndex;
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	// first find the index where we pull from.
	queueIndex = videoQueue.head; //(videoQueue.tail + videoQueue.size - 1) % videoQueue.size;
	*jpgSize = videoQueue.elements[queueIndex].imageSize;
	if(*jpgSize > JPG_IMAGE_BUFFER_SIZE) {*jpgSize = JPG_IMAGE_BUFFER_SIZE;}
	memcpy(jpg, videoQueue.elements[queueIndex].image, *jpgSize);

	xSemaphoreGive( xLoggingMutex);
}
// this returns a pointer to most recent video image and it's size (in bytes).
// note that you need to free this when you're done.
uint8_t *getVideoElementPtr(uint32_t *jpgSize)
{
	uint8_t *jpg;
	uint32_t queueIndex;
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	// first find the index where we pull from.
	queueIndex = videoQueue.head; //(videoQueue.tail + videoQueue.size - 1) % videoQueue.size;
	*jpgSize = videoQueue.elements[queueIndex].imageSize;
	if(*jpgSize > JPG_IMAGE_BUFFER_SIZE) {*jpgSize = JPG_IMAGE_BUFFER_SIZE;}
	if(jpgSize > 0){
		jpg = malloc(*jpgSize);
		memcpy(jpg, videoQueue.elements[queueIndex].image, *jpgSize);
	}else{
		jpg = NULL;
	}
	xSemaphoreGive( xLoggingMutex);
	return jpg;
}

// this returns the oldest video image and it's size (in bytes).
void getOldestVideoElement(uint32_t *jpgSize, uint8_t *jpg)
{
	uint32_t queueIndex;
	// first find the index where we pull from.
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	queueIndex = videoQueue.tail; //(videoQueue.tail + videoQueue.size - 1) % videoQueue.size;
	if(queueIndex != videoQueue.head){
		*jpgSize = videoQueue.elements[queueIndex].imageSize;
		if(*jpgSize > JPG_IMAGE_BUFFER_SIZE) {*jpgSize = JPG_IMAGE_BUFFER_SIZE;}
		memcpy(jpg, videoQueue.elements[queueIndex].image, *jpgSize);
		queueIndex++;
		if(queueIndex > videoQueue.size){queueIndex = 0;}
	}else{
		*jpgSize = 0;
	}
	videoQueue.tail = queueIndex;
	xSemaphoreGive( xLoggingMutex);
}





void enqueueRadarElement(int16_t *angle, int16_t *distance, int16_t *magnitude, uint16_t numbElements){

	int i;

	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	for(i = 0; i < numbElements; i++){
		radarQueue.head++;
		if(radarQueue.head >= radarQueue.size){ radarQueue.head = 0;}
		if(radarQueue.head == radarQueue.tail){
			radarQueue.tail++;
			if(radarQueue.tail >= radarQueue.size){
				radarQueue.tail = 0;
			}
		}

		radarQueue.elements[radarQueue.head].angle = (int8_t) angle[i];
		radarQueue.elements[radarQueue.head].distance = (uint8_t) distance[i];
		radarQueue.elements[radarQueue.head].magnitude = (uint8_t) magnitude[i];

	}
	xSemaphoreGive( xLoggingMutex);

}

void enqueueCameraElement(struct STR_Jack_CameraTarget *cameraTargetList, uint16_t numbElements){

	int i;

	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	for(i = 0; i < numbElements; i++){
		cameraQueue.head++;
		if(cameraQueue.head >= cameraQueue.size){ cameraQueue.head = 0;}
		if(cameraQueue.head == cameraQueue.tail){
			cameraQueue.tail++;
			if(cameraQueue.tail >= cameraQueue.size){
				cameraQueue.tail = 0;
			}
		}
		memcpy(&cameraQueue.elements[cameraQueue.head], &cameraTargetList[i], sizeof(struct STR_Jack_CameraTarget));

	}
	xSemaphoreGive( xLoggingMutex);

}



// this pulls 'numbElements' from the radarQueue and coppies it in to radarQueueHistory.
// radarQueueHistory must be big enough to hold these elements.
void getOldestRadarElementHistory(uint32_t numbElements, struct STR_LOGGING_RadarElement *radarQueueHistory)
{
	uint32_t queueIndex, i;
	// first find the index where we start to pull from.
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	queueIndex = radarQueue.tail; //(radarQueue.tail + radarQueue.size - numbElements) % radarQueue.size;

	for(i = 0; i < numbElements; i++){
		if(queueIndex != radarQueue.head){
			memcpy(&(radarQueueHistory[i]), &radarQueue.elements[queueIndex], sizeof(struct STR_LOGGING_RadarElement));
			queueIndex++;
			if(queueIndex >= radarQueue.size) {queueIndex = 0;}
		}else{
			memset(&(radarQueueHistory[i]), 0, sizeof(struct STR_LOGGING_RadarElement));	// over consuming.
		}
	}
	radarQueue.tail = queueIndex;
	xSemaphoreGive( xLoggingMutex);

}

// this pulls 'numbElements' from the radarQueue and coppies it in to radarQueueHistory.
// radarQueueHistory must be big enough to hold these elements.
void getRadarElementHistory(uint32_t numbElements, struct STR_LOGGING_RadarElement *radarQueueHistory)
{
	uint32_t queueIndex, i;
	// first find the index where we start to pull from.
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);



	queueIndex = radarQueue.head + radarQueue.size;
	queueIndex = queueIndex - numbElements;
	queueIndex = queueIndex % radarQueue.size;

	i = 0;
	do{

			memcpy(&(radarQueueHistory[i]), &radarQueue.elements[queueIndex], sizeof(struct STR_LOGGING_RadarElement));
			queueIndex++;
			i++;
			if(queueIndex >= radarQueue.size) {queueIndex = 0;}
	}while(queueIndex != radarQueue.head);


	xSemaphoreGive( xLoggingMutex);

}
// this pulls 'numbElements' from the cameraQueue and coppies it in to cameraQueueHistory.
// cameraQueueHistory must be big enough to hold these elements.
void getCameraElementHistory(uint32_t numbElements, struct STR_Jack_CameraTarget *cameraQueueHistory)
{
	uint32_t queueIndex, i;
	// first find the index where we start to pull from.
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);



	queueIndex = cameraQueue.head + cameraQueue.size;
	queueIndex = queueIndex - numbElements;
	queueIndex = queueIndex % cameraQueue.size;

	i = 0;
	do{

			memcpy(&(cameraQueueHistory[i]), &cameraQueue.elements[queueIndex], sizeof(struct STR_Jack_CameraTarget));
			queueIndex++;
			i++;
			if(queueIndex >= cameraQueue.size) {queueIndex = 0;}
	}while(queueIndex != cameraQueue.head);


	xSemaphoreGive( xLoggingMutex);

}

void enqueuePIRElement(uint8_t *pir0, uint8_t *pir1, int numbElements){

	int i;
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	for(i = 0; i < numbElements; i++){
		pirQueue.head++;
		if(pirQueue.head >= pirQueue.size){ pirQueue.head = 0;}
		if(pirQueue.head == pirQueue.tail){
			pirQueue.tail++;
			if(pirQueue.tail >= pirQueue.size){
				pirQueue.tail = 0;
			}
		}
		pirQueue.elements[0][pirQueue.head] = pir0[i];
		pirQueue.elements[1][pirQueue.head] = pir1[i];
	}
	xSemaphoreGive( xLoggingMutex);
}
// THis gets the OLDEST PIR elements.  Also moves up the queue's tail.
void getOldestPIRElementHistory_uint8(uint32_t numbElements,  uint8_t *pir0, uint8_t *pir1)
{
	uint32_t queueIndex, i;
	// first find the index where we start to pull from.
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	queueIndex = pirQueue.tail; // WTF was I thinking here?? -> (pirQueue.tail + pirQueue.size - numbElements) % pirQueue.size;

	for(i = 0; i < numbElements; i++){
		if(queueIndex != pirQueue.head){
			pir0[i] = (uint8_t) pirQueue.elements[0][queueIndex];
			pir1[i] = (uint8_t) pirQueue.elements[1][queueIndex];
			queueIndex++;
			if(queueIndex >= pirQueue.size) {queueIndex = 0;}
		}
		else{
			pir0[i] = 0;		// over consuming.
			pir1[i] = 0;
		}
	}
	pirQueue.tail = queueIndex;
	xSemaphoreGive( xLoggingMutex);

}
// this gets the most recent PIR elements.
void getPIRElementHistory_uint8(uint32_t numbElements,  uint8_t *pir0, uint8_t *pir1)
{
	uint32_t queueIndex, i;
	// first find the index where we start to pull from.
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	queueIndex = pirQueue.head + pirQueue.size;
	queueIndex = queueIndex - numbElements;
	queueIndex = queueIndex % pirQueue.size;
	i = 0;
	do{
		pir0[i] = (uint8_t) pirQueue.elements[0][queueIndex];
		pir1[i] = (uint8_t) pirQueue.elements[1][queueIndex];
		queueIndex++;
		i++;
		if(queueIndex >= pirQueue.size) {queueIndex = 0;}
	}while(queueIndex != pirQueue.head);


	xSemaphoreGive( xLoggingMutex);

}
void enqueueAudioElement(int16_t *ch0, int16_t *ch1, int16_t *ch2, int numbElements){

	int i;
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	for(i = 0; i < numbElements; i++){
		audioQueue.head++;
		if(audioQueue.head >= audioQueue.size){ audioQueue.head = 0;}
		if(audioQueue.head == audioQueue.tail){
			audioQueue.tail++;
			if(audioQueue.tail >= audioQueue.size){
				audioQueue.tail = 0;
			}
		}
		audioQueue.elements[audioQueue.head][0] = ch0[i];
		audioQueue.elements[audioQueue.head][1] = ch1[i];
		audioQueue.elements[audioQueue.head][2] = ch2[i];
	}
	xSemaphoreGive( xLoggingMutex);
}

void enqueueAccelElement(int16_t *x, int16_t *y, int16_t *z, int numbElements){

	int i;
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	for(i = 0; i < numbElements; i++){
		accelQueue.head++;
		if(accelQueue.head >= accelQueue.size){ accelQueue.head = 0;}
		if(accelQueue.head == accelQueue.tail){
			accelQueue.tail++;
			if(accelQueue.tail >= accelQueue.size){
				accelQueue.tail = 0;
			}
		}
		accelQueue.elements[0][accelQueue.head] = x[i];
		accelQueue.elements[1][accelQueue.head] = y[i];
		accelQueue.elements[2][accelQueue.head] = z[i];
	}
	xSemaphoreGive( xLoggingMutex);
}

void getAccelElementHistory_16BitXYZ(uint32_t numbElements,  int16_t *xyzData)		// xyz data is 3x numbElements.
{
	uint32_t queueIndex, i,j;
	// first find the index where we start to pull from.
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

	queueIndex = (accelQueue.head + accelQueue.size - numbElements) % accelQueue.size;
	j = 0;
	for(i = 0; i < numbElements; i++){
		xyzData[j++] = accelQueue.elements[0][queueIndex];
		xyzData[j++] = accelQueue.elements[1][queueIndex];
		xyzData[j++] = accelQueue.elements[2][queueIndex];

		queueIndex++;
		if(queueIndex >= accelQueue.size) {queueIndex = 0;}
	}

	xSemaphoreGive( xLoggingMutex);

}


#define TIMEOUT_ATTEMPTS 5
#if(0)
int writeMsg_withTimeout(struct netconn *conn, void *dataptr, size_t size, u8_t apiflags){
	size_t index, byteCounter, bytesToWrite, bytesWritten;
	bytesToWrite = size;
	byteCounter = 0;
	int timeoutCtr = TIMEOUT_ATTEMPTS;
	int err = 0;
	do{
		index = size - byteCounter;
		if(index > 1460){//61440) {
			index = 1460;//61440;
		}

		bytesWritten = 0;
		err = netconn_write_partly(conn, (void *)(dataptr + byteCounter), index, apiflags, &bytesWritten);


			if(bytesWritten == 0){
				if(timeoutCtr-- == 0){
					err = error_tIMEOUT;
					break;
				}
				vTaskDelay(5 / portTICK_PERIOD_MS );
				err = NO_ERROR;	// we can deal with this error.
			}
			if(err == ERR_WOULDBLOCK){
				if(timeoutCtr-- == 0){
					break;
				}
				vTaskDelay(5 / portTICK_PERIOD_MS );
				err = NO_ERROR;	// we can deal with this error.
			}
			if(err == ERR_RST){
				break;			//
			}

		byteCounter += bytesWritten;
// NO!!!!!!!!!!!!!!!!!!!!  		timeoutCtr = TIMEOUT_ATTEMPTS;
	}while(byteCounter < bytesToWrite);

	return err;
}
#endif

#if(0)
int writeMsg_withTimeout_socket_old(const int sock, uint8_t *dataptr, size_t size)
{
    int written;
    int timeout = TIMEOUT_ATTEMPTS;

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.

            int to_write = size;
            while ((to_write > 0) && (timeout >0))
            {
            	written = send(sock, dataptr + (size - to_write), to_write, 0);;
                if (written < 0) {

                	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
    				PRINTF("Error occurred during sending: errno %d\r\n", errno);
    				xSemaphoreGive( xPrintMutex );

    				vTaskDelay(5 / portTICK_PERIOD_MS );
    				timeout--;
    				continue;
                }
                to_write -= written;

            }

	if(timeout == 0){
		return error_tIMEOUT;  // Failed to retransmit, giving up
	}
	return NO_ERROR;

}
#endif

int writeMsg_withTimeout_socket(Socket *sock, uint8_t *dataptr, size_t size)
{
    int written;
    int timeout = TIMEOUT_ATTEMPTS;
    error_t error;
            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.

            int to_write = size;
            while ((to_write > 0) && (timeout >0))
            {
            	//written = send(sock, dataptr + (size - to_write), to_write, 0);;
            	error = socketSend(sock, dataptr + (size - to_write), to_write, &written, 0);
                if (written < 0) {

                	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
    				PRINTF("Error occurred during sending: errno %d\r\n", error);
    				xSemaphoreGive( xPrintMutex );

    				vTaskDelay(5 / portTICK_PERIOD_MS );
    				timeout--;
    				continue;
                }
                to_write -= written;

            }

	if(timeout == 0){
		return ERROR_TIMEOUT;  // Failed to retransmit, giving up
	}
	return error;

}
#if(0)
int writeMsg_withTimeout_socketSendTo(const int sock, uint8_t *dataptr, size_t size, struct sockaddr *clientAddr )
{
    int written;
    int timeout = TIMEOUT_ATTEMPTS;
//    socklen_t addrLen;
            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.

            int to_write = size;
            while ((to_write > 0) && (timeout >0))
            {
				written = sendto(sock, dataptr + (size - to_write), to_write, 0, clientAddr, sizeof(struct sockaddr_in));
//            	written =  send(sock, dataptr + (size - to_write), to_write, 0);;
                if (written < 0) {

                	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
    				PRINTF("Error occurred during sending: errno %d\r\n", errno);
    				xSemaphoreGive( xPrintMutex );

    				vTaskDelay(5 / portTICK_PERIOD_MS );
    				timeout--;
    				continue;
                }
                to_write -= written;

            }

	if(timeout == 0){
		return error_tIMEOUT;  // Failed to retransmit, giving up
	}
	return NO_ERROR;

}
#endif










//https://www.xilinx.com/video/soc/networking-with-lwip-focused-free-rtos.html
//https://github.com/dreamcat4/lwip/blob/master/contrib/apps/socket_examples/socket_examples.c
//https://github.com/espressif/esp-idf/blob/4d14c2ef2d9d08cd1dcbb68a8bb0d76a666e2b4b/examples/protocols/sockets/tcp_server/main/tcp_server.c





#define LOGGING_BUFFER_SIZE	(1460 * 10)
void checkRGMIIStatus(void);
#if(0)
void logging_task(void *pvParameters)
{

	TickType_t wakeTimer = 0;
	uint8_t buffer[44];
	uint32_t  transmitPrescaller;
	size_t elementCount;
	int socketStatusCopy;
	int rxBufferIndex = 0;
	int len;
	uint32_t	temp32;


	error_t err = NO_ERROR;

	do {
		xTaskDelayUntil( &wakeTimer, 	1000 / portTICK_PERIOD_MS );

		xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
			socketStatusCopy = socketStatus ;
		xSemaphoreGive( xLoggingMutex);
//		checkRGMIIStatus();
		while(socketStatusCopy != 0){
			xTaskDelayUntil( &wakeTimer, 	100 / portTICK_PERIOD_MS );
			transmitPrescaller++;
			if(transmitPrescaller >= 20){
				transmitPrescaller = 0;

				errno = 0;

				err = sendJack_2_3();
				if(err == NO_ERROR){
					wdog_networkActivity |= 2;
				}

			}


				elementCount = getQueueLength(&rxBuffer);
				if(elementCount >= 44){					// 44 is minimum msg size.
					if(elementCount > sizeof(buffer)){
						elementCount = sizeof(buffer);
					}
					elementCount = inspectQueueTXRX(&rxBuffer, buffer, elementCount);

					rxBufferIndex = 0;
//					while (elementCount > 0){

					if( ((struct STR_LOGGING_MSG_Headder *)buffer)->STX == 0x000034E0)
					{
						len = ((struct STR_LOGGING_MSG_Headder *)buffer)->LEN + 8;		// 8 additional bytes for SIG and LEN.
						if(len < elementCount)
						{
	// THIS CONDITION NOT TESTED!!!!
							// more than our msg is in the quueue.
							temp32 = (uint32_t)buffer[rxBufferIndex + len];		//	check the next msg to ensure alignment.
							if(temp32 == 0x000034E0){

								elementCount = handleReceive(&buffer[rxBufferIndex], elementCount);
									// weird, we process the alarm... THEN dequeue it.  Odd, but it works out.
								if(elementCount <0){
									// next msg didn't pass alignment.  Ditch this msg by just dequeueing the 1st byte.
									elementCount = getQueueTXRX(&rxBuffer, buffer, 1);
								}else{
									elementCount = getQueueTXRX(&rxBuffer, buffer, len + 4);
								}




							}else{
								// next msg didn't pass alignment.  Ditch this msg by just dequeueing the 1st byte.
								elementCount = getQueueTXRX(&rxBuffer, buffer, 1);
							}
						}else if( len == elementCount){
							// we have exactly the number of bytes in the msg buffer that we need.  So we'll call algnment good.
							elementCount = getQueueTXRX(&rxBuffer, buffer, len);

							elementCount = handleReceive(&(buffer[rxBufferIndex]), elementCount);

						}else {
	// THIS CONDITION NOT TESTED!!!!
							// we don't have enough bytes in our queueTo satisfy length.
							if(len > 560)	{//560 serves as our max msg size right now.
								// our msg didn't pass alignment.  Ditch this msg by just dequeueing the 1st byte.
								elementCount = getQueueTXRX(&rxBuffer, buffer, 1);

							}
						}

					}


//				}
			}



		};

	}while(1);


}
#endif

#define PORT                        10005
#if(0)
void tcp_logging_task(void *pvParameters)
{
	TickType_t wakeTimer = 0;



	uint8_t *tcpLoggingBuffer;
	tcpLoggingBuffer = malloc(LOGGING_BUFFER_SIZE);

	error_t err = NO_ERROR;
	uint32_t i;
	ssize_t receiveCount, transmitCount;
	int opt;
    char addr_str[128];
    int addr_family = (int)pvParameters;

    struct sockaddr_storage dest_addr;
    int listen_sock, loggingSocketHandle;
	struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);

    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
	dest_addr_ip4->sin_family = AF_INET;
	dest_addr_ip4->sin_port = htons(PORT);

	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
			memset(&accelQueue, 0, sizeof(accelQueue));
			memset(&audioQueue, 0, sizeof(audioQueue));
			memset(&pirQueue, 0, sizeof(pirQueue));
			memset(&radarQueue, 0, sizeof(radarQueue));
			memset(&videoQueue, 0, sizeof(videoQueue));
			memset(&cameraQueue, 0, sizeof(cameraQueue));
			accelQueue.size = 16000;
			audioQueue.size = 320000;
			pirQueue.size = PIR_QUEUE_SIZE;
			radarQueue.size = 800;
			videoQueue.size = 20;
			cameraQueue.size = 1600;

			txBuffer.head = 0;
			txBuffer.tail = 0;
			txBuffer.size = TX_BUFFER_SIZE;
			txBuffer.elements = malloc(TX_BUFFER_SIZE);

			rxBuffer.head = 0;
			rxBuffer.tail = 0;
			rxBuffer.size = RX_BUFFER_SIZE;
			rxBuffer.elements = malloc(RX_BUFFER_SIZE);

			socketStatus = 0;
	xSemaphoreGive( xLoggingMutex);



	//	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 1000 );
	LWIP_UNUSED_ARG(pvParameters);
	/* Store the handle of the calling task. */


	/* Main loop. Get sensor data and send via TCP */
	while (1)
	{

		listen_sock = socket(addr_family, SOCK_STREAM, IPPROTO_TCP);

		if (listen_sock < 0) {
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Logging - Unable to create socket: errno %d\r\n", errno);
			xSemaphoreGive( xPrintMutex );
			break; //goto CLEAN_UP;
		}
		opt = 1;
		err = setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
		if(err != NO_ERROR){PRINTF("Logging - socket set opt Fail\r\n");}

		// I'm not actually sure if we need linger turned on.  But it works.
		const struct linger linger = {.l_onoff = 1, .l_linger = 5};
		err = setsockopt(listen_sock, SOL_SOCKET, SO_LINGER, &linger, sizeof(linger));
		if(err != NO_ERROR){PRINTF("socket set opt Fail\r\n");}

		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Logging Socket created!\r\n");
		xSemaphoreGive( xPrintMutex );

		err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if (err != 0) {
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Logging - Socket unable to bind: errno %d\r\n", errno);
			PRINTF("IPPROTO: %d", addr_family);
			xSemaphoreGive( xPrintMutex );
			break;  //goto CLEAN_UP;
		}

		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Logging Socket bound, port %d\r\n", PORT);
		xSemaphoreGive( xPrintMutex );


		err = listen(listen_sock, 1);
		if (err != 0) {
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Error occurred during listen - Logging Socket: errno %d\r\n", errno);
			xSemaphoreGive( xPrintMutex );
			break;//goto CLEAN_UP;
		}


		while (1) {

			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Logging Socket listening\r\n");
			xSemaphoreGive( xPrintMutex );

	        loggingSocketHandle = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
	        if (loggingSocketHandle < 0) {
		    	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
				PRINTF("Unable to accept connection: - Logging Socket: errno %d\r\n", errno);
				xSemaphoreGive( xPrintMutex );
	            break;
	        }

			// Set tcp keepalive option
	        int keepAlive = 1;	// 1= enable keep alive
	        int keepIdle = 5;	// 5 = send first keepalive probe after 5 seconds of idleness;
	        int keepInterval = 3;// 3 = send subsequent keepalive probels after 3 seconds.
	        int keepCount = 3; 	// 3 = timeout after 3 failed probes.;
			err = setsockopt(loggingSocketHandle, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
			err = setsockopt(loggingSocketHandle, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
			err = setsockopt(loggingSocketHandle, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
			err = setsockopt(loggingSocketHandle, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

			 // enable send timeout.
		    struct timeval sendTimeout, recvTimeout;
		    sendTimeout.tv_sec = 2;
		    sendTimeout.tv_usec = 0;		// 2s
		    recvTimeout.tv_sec = 0;
		    recvTimeout.tv_usec = 1000;	//0.001 S
			//{
		    //  long    tv_sec;         /* seconds */
		    //  long    tv_usec;        /* and microseconds */
		    //};
	        err = setsockopt(loggingSocketHandle, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));	// enable send timeout.
	        err = setsockopt(loggingSocketHandle, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));	// enable rx timeout.

	        // Convert ip address to string
	        if (source_addr.ss_family == PF_INET) {
	            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
	        }
	        i = ((((struct sockaddr_in *)&source_addr)->sin_port) & 0x00FF) * 256;
	        i = i + ((((struct sockaddr_in *)&source_addr)->sin_port) >> 8);



	    	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Logging Socket accepted ip address: %s : %d\r\n", addr_str, i);
			xSemaphoreGive( xPrintMutex );

			do {
				xTaskDelayUntil( &wakeTimer, 	100 / portTICK_PERIOD_MS );
				xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
					socketStatus = 1;
				xSemaphoreGive( xLoggingMutex);
				errno = 0;
				err = NO_ERROR;
				transmitCount = getQueueLength(&txBuffer);

				while((transmitCount >0) && (err == NO_ERROR)){
					if(transmitCount > 0){

						if(transmitCount > LOGGING_BUFFER_SIZE){
							transmitCount = LOGGING_BUFFER_SIZE;
						}
						transmitCount = getQueueTXRX(&txBuffer, tcpLoggingBuffer, transmitCount);

						err = writeMsg_withTimeout_socket(loggingSocketHandle, (uint8_t *)tcpLoggingBuffer, transmitCount);
					}
					transmitCount = getQueueLength(&txBuffer);
					if(err == NO_ERROR){
						wdog_networkActivity |= 2;

					}
					if(transmitCount > 0){
						vTaskDelay(0);
					}
				};


				errno = 0;
				i = 0;
				receiveCount = recv(loggingSocketHandle, tcpLoggingBuffer, LOGGING_BUFFER_SIZE, 0);

				if(receiveCount < 0){
					if(errno == ENOTCONN){
						err = -1;	// ENOTCONN means the other end has blown away the connection.  We need to reset.
						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
							PRINTF("Logging port closed: errno %d\r\n", errno);
						xSemaphoreGive(xPrintMutex);
					}
					else if(errno != EAGAIN){
//						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//						PRINTF("Logging Error occurred during receiving: errno %d\r\n", errno);
//						xSemaphoreGive( xPrintMutex );
					}
				} else if (receiveCount == 0) {
						// nothing to receive.
				} else {
//					xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//					PRINTF("RX Data: %i bytes.\r\n", receiveCount);
//					xSemaphoreGive(xPrintMutex);

					while(receiveCount >0){
						receiveCount = receiveCount - enqueueTXRX(&rxBuffer, tcpLoggingBuffer, receiveCount);
						if(receiveCount > 0 ){
							vTaskDelay(10);
						}
					}
				}
			}while(err == NO_ERROR);
			xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
				socketStatus = 0;
			xSemaphoreGive( xLoggingMutex);

	        shutdown(loggingSocketHandle, SHUT_RDWR);
	        close(loggingSocketHandle);
	    }

//	CLEAN_UP:
		xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
			socketStatus = 0;
		xSemaphoreGive( xLoggingMutex);

		shutdown(listen_sock, SHUT_RDWR);	//TS
	    close(listen_sock);

	}
	free(tcpLoggingBuffer);
}
#endif
struct STR_Jack_MSG_2_3 jackMessage_2_3;
struct STR_LOGGING_RadarElement radarQueueHistory[80];		// 2 seconds worht of data (10 targets each), at 4Hz update rate = 80 elements.

// this sends a JACK msg 2.3; plot data for sensors.
int sendJack_2_3(void){//struct netconn *loggingNetconn){
	uint32_t i;
	static int jackMessageSequence = 0;
	int err = 0;

	memset(&jackMessage_2_3, 0, sizeof(jackMessage_2_3));
	jackMessage_2_3.STX = 0x000034E0;
	jackMessage_2_3.LEN = sizeof(jackMessage_2_3) - (sizeof(jackMessage_2_3.STX) + sizeof(jackMessage_2_3.LEN));
	jackMessage_2_3.TYPE = 0x00010003;
	jackMessage_2_3.VER = 0x00000000;
	jackMessage_2_3.NODE = node_num;
	jackMessage_2_3.TIME = jackMessageSequence++;		// TODO.  I know this isn't Time.  But it's sequence... and that's the best we're going to do for now.

	memcpy(&(jackMessage_2_3.radarMessage_cartesian[0]), radarDataForJack_cartesian, sizeof(radarDataForJack_cartesian));

	jackMessage_2_3.leftPIRMagConf.magnitude = pirDataForJack[0];
	jackMessage_2_3.leftPIRMagConf.confidence = pirConfidenceForLogging;

	jackMessage_2_3.rightPIRMagConf.magnitude = pirDataForJack[1];
	jackMessage_2_3.rightPIRMagConf.confidence = pirConfidenceForLogging;

	jackMessage_2_3.accelMagConf.magnitude = accelDataForJack;
	jackMessage_2_3.accelMagConf.confidence = accelConfidenceForLogging;

	jackMessage_2_3.cameraMagConf.magnitude = 0;
	jackMessage_2_3.cameraMagConf.confidence = 0;

	jackMessage_2_3.temperature = processorTemperature;	//0x1234;
	jackMessage_2_3.packingByte[0] = 0x5A;
	jackMessage_2_3.packingByte[1] = 0xA5;

	getRadarElementHistory(80, &(radarQueueHistory[0]));
	for(i = 0; i < 80; i++){
		jackMessage_2_3.rawRadar[i].angle = (int8_t) radarQueueHistory[i].angle;
		jackMessage_2_3.rawRadar[i].distance = (uint8_t) radarQueueHistory[i].distance;
		jackMessage_2_3.rawRadar[i].magnitude = (uint8_t) radarQueueHistory[i].magnitude;
	}
	getPIRElementHistory_uint8(40,  jackMessage_2_3.leftPir, jackMessage_2_3.rightPir);

	getAccelElementHistory_16BitXYZ(1600, jackMessage_2_3.accelData );

	getCameraElementHistory(80, jackMessage_2_3.cameraTargets);

	getVideoElement(&jackMessage_2_3.jpegSize, jackMessage_2_3.jpegData);
//	jackMessage_2_3.jpegSize = JPG_IMAGE_BUFFER_SIZE;  // debug... youll need to remove this line.
	jackMessage_2_3.LEN = sizeof(jackMessage_2_3) -
							(sizeof(jackMessage_2_3.STX) + sizeof(jackMessage_2_3.LEN)) -
							(sizeof(jackMessage_2_3.jpegData)) +
							jackMessage_2_3.jpegSize;


	accelConfidenceForLogging = 0;	// these are just used as Flags right now.  I.e. Alarm flags.
	pirConfidenceForLogging = 0;
	radarConfidenceForLogging = 0;

	//err |= netconn_write(loggingNetconn, (uint8_t *)&(jackMessage), (sizeof(jackMessage)), NETCONN_NOCOPY);
		//err |= netconn_write(loggingNetconn, (uint8_t *)&(jackMessage), (sizeof(jackMessage)), NETCONN_NOCOPY);
#if(0)
	err |= writeMsg_withTimeout(loggingNetconn, (uint8_t *)&(jackMessage_2_3), (jackMessage_2_3.LEN + (sizeof(jackMessage_2_3.STX) + sizeof(jackMessage_2_3.LEN))),NETCONN_COPY);
#else
//	err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(jackMessage_2_3), (jackMessage_2_3.LEN + (sizeof(jackMessage_2_3.STX) + sizeof(jackMessage_2_3.LEN))));
	err |= addMessageToQueues_blocking(		 (uint8_t *)&(jackMessage_2_3), (jackMessage_2_3.LEN + (sizeof(jackMessage_2_3.STX) + sizeof(jackMessage_2_3.LEN))));

	#endif



	return err;

}







const uint8_t wavHeadder[0x2C] = {0x52, 0x49, 0x46, 0x46, 0x2C, 0x4C, 0x1D, 0x00, 0x57, 0x41, 0x56, 0x45, 0x66, 0x6d, 0x74, 0x20,
								  0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x80, 0x3e, 0x00, 0x00, 0x00, 0x7d, 0x00, 0x00,
								  0x02, 0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0x4C, 0x1D, 0x00
};
uint32_t loggingMessageSequence = 0;
#if(0)
int handleRX_2_3(uint8_t *buffer, int msgLenght);
int handleTX_3_9(uint8_t *data, int msgLength);
int handleRX_3_9(uint8_t *buffer, int msgLenght);
int handleReceive(uint8_t *buffer, int msgLenght){

	int err = NO_ERROR;
//	uint32_t	length;
	int type;
//	int ver;
	uint8_t *data;

	// TODO error checking.

	if(msgLenght < 44) {
		return -1;	// 44 is our min msg size at this point.
	}

	type = ((struct STR_LOGGING_MSG_Headder *)buffer)->TYPE & 0x00FFFFFF;	// strip leading 0x80
//	ver = ((struct STR_LOGGING_MSG_Headder *)buffer)->VER;					// version.  for future use.
	data = buffer + 8;

	switch(type){
					case(0x10003):	// msg 2.3	// logging msg.
						err |= handleRX_2_3(data, msgLenght);
					break;
					case(0x800C):	// msg 3.9	// SEND Req. device configuration
						err |= handleTX_3_9(data, msgLenght);
					break;
					case(0x000C):	// msg 3.9	// device configuration
						err |= handleRX_3_9(data, msgLenght);
					break;


					default:
						return -1;
	}
	return err;
}

int handleTX_3_9(uint8_t *data, int msgLength){

	int err;
	struct STR_ConfigParam newMsg;

	memset(&newMsg,0,sizeof(newMsg));

	err = addMessageToQueues_blocking((uint8_t *)&(newMsg), (sizeof(newMsg)));

	return err;

};

int handleRX_3_9(uint8_t *buffer, int msgLenght){
	return 0;
}
int handleRX_2_3(uint8_t *buffer, int msgLenght){

	int err;
//	uint32_t lenght;
	struct STR_LOGGING_MSG_Headder loggingMessage;
	static int sent_frames;
	int i,j;

	if(buffer[sizeof(loggingMessage) + 4 - 1] == 0x00){
		memset(&loggingMessage, 0, sizeof(loggingMessage));
		loggingMessage.STX = 0x000034E0;
		loggingMessage.LEN = sizeof(loggingMessage) - (sizeof(loggingMessage.STX) + sizeof(loggingMessage.LEN));
		loggingMessage.TYPE = 0x00000003;
		loggingMessage.VER = 0x00010001;
		loggingMessage.NODE = node_num;
		loggingMessage.TIME = loggingMessageSequence++;		// TODO.  I know this isn't Time.  But it's sequence... and that's the best we're going to do for now.
		err = NO_ERROR;

//						err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(loggingMessage), (sizeof(loggingMessage)));
		err |= addMessageToQueues_blocking((uint8_t *)&(loggingMessage), (sizeof(loggingMessage)));

	}else if(buffer[sizeof(loggingMessage) + 4 - 1] == 0x01){
// msgLength = 44.
		msgLenght = 44;
		memset(&loggingMessage, 0, sizeof(loggingMessage));
		loggingMessage.STX = 0x000034E0;

		loggingMessage.LEN = sizeof(loggingMessage) - (sizeof(loggingMessage.STX) + sizeof(loggingMessage.LEN));

		loggingMessage.TYPE = 0x00000003;
		loggingMessage.VER = 0x00010001;
		loggingMessage.NODE = node_num;
		loggingMessage.TIME = loggingMessageSequence++;		// TODO.  I know this isn't Time.  But it's sequence... and that's the best we're going to do for now.
		loggingMessage.SUB_MSG_TYPE = 0x00000001;
		err = NO_ERROR;


		xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);

		memcpy(&radarQueueCopy, &radarQueue, (sizeof(radarQueue)));
		memcpy(&pirQueueCopy, &pirQueue, (sizeof(pirQueue)));
		memcpy(&accelQueueCopy, &accelQueue, (sizeof(accelQueue)));
		memcpy(&audioQueueCopy, &audioQueue, (sizeof(audioQueue)));
		memcpy(&cameraQueueCopy, &cameraQueue, (sizeof(cameraQueue)));
		memcpy(&videoQueueCopy, &videoQueue, (sizeof(videoQueue)));
		xSemaphoreGive( xLoggingMutex);

//						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//						PRINTF("Logging task: Sending CSV Data.  \r\n");
//						xSemaphoreGive(xPrintMutex);


		loggingMessage.LEN += (radarQueueCopy.size) * (sizeof(uint8_t) * 3);
		loggingMessage.LEN += (pirQueueCopy.size) * (sizeof(uint8_t)) * 2;
		loggingMessage.LEN += (accelQueueCopy.size) * (sizeof(uint16_t)) * 3;
		loggingMessage.LEN += (cameraQueueCopy.size) * (sizeof(struct STR_Jack_CameraTarget));


//		err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(loggingMessage), (sizeof(loggingMessage)));
		err |= addMessageToQueues_blocking((uint8_t *)&(loggingMessage), sizeof(loggingMessage));
		if(err != NO_ERROR){
			return msgLenght;}


// Radar

//						err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(radarQueueCopy.elements[radarQueueCopy.head]), (radarQueueCopy.size - radarQueueCopy.head) * (sizeof(uint8_t)) * 3);
		err |= addMessageToQueues_blocking(      (uint8_t *)&(radarQueueCopy.elements[radarQueueCopy.head]), (radarQueueCopy.size - radarQueueCopy.head) * (sizeof(uint8_t)) * 3);
		if(err != NO_ERROR){	return msgLenght;}
		if(radarQueueCopy.head != 0){

//							err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(radarQueueCopy.elements[0]), (radarQueueCopy.head) * (sizeof(uint8_t)) * 3);
			err |= addMessageToQueues_blocking((uint8_t *)&(radarQueueCopy.elements[0]), (radarQueueCopy.head) * (sizeof(uint8_t)) * 3);

		}
// PIR

//						err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(pirQueueCopy.elements[0][pirQueueCopy.head]), (pirQueueCopy.size - pirQueueCopy.head) * (sizeof(uint8_t)) );
		err |= addMessageToQueues_blocking(		 (uint8_t *)&(pirQueueCopy.elements[0][pirQueueCopy.head]), (pirQueueCopy.size - pirQueueCopy.head) * (sizeof(uint8_t)) );
		if(err != NO_ERROR){	return msgLenght;}
		if(pirQueueCopy.head != 0){

//							err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(pirQueueCopy.elements[0][0]), (pirQueueCopy.head) * (sizeof(uint8_t)));
			err |= addMessageToQueues_blocking(		 (uint8_t *)&(pirQueueCopy.elements[0][0]), (pirQueueCopy.head) * (sizeof(uint8_t)));
		}


//						err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(pirQueueCopy.elements[1][pirQueueCopy.head]), (pirQueueCopy.size - pirQueueCopy.head) * (sizeof(uint8_t)));
		err |= addMessageToQueues_blocking(		 (uint8_t *)&(pirQueueCopy.elements[1][pirQueueCopy.head]), (pirQueueCopy.size - pirQueueCopy.head) * (sizeof(uint8_t)));
		if(err != NO_ERROR){	return msgLenght;}
		if(pirQueueCopy.head != 0){

		//	err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(pirQueueCopy.elements[1][0]), (pirQueueCopy.head) * (sizeof(uint8_t)));
			err |= addMessageToQueues_blocking(		 (uint8_t *)&(pirQueueCopy.elements[1][0]), (pirQueueCopy.head) * (sizeof(uint8_t)));
		}


// Accel

		int16_t *bufferPtr16Bit;

//						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//						PRINTF("Logging task: Sending Data... \r\n");
//						xSemaphoreGive(xPrintMutex);

		bufferPtr16Bit = malloc(accelQueueCopy.size * 2 * 3);	// x,y,x * 2 for 16bit.


		int queueIndex;

		queueIndex = accelQueue.head + 1;
		if(queueIndex > accelQueue.size){queueIndex =0;}

		j = 0;
		for(i = 0; i < accelQueueCopy.size; i++){
			bufferPtr16Bit[j++] = accelQueueCopy.elements[0][queueIndex];
			bufferPtr16Bit[j++] = accelQueueCopy.elements[1][queueIndex];
			bufferPtr16Bit[j++] = accelQueueCopy.elements[2][queueIndex];

			queueIndex++;
			if(queueIndex >= accelQueue.size) {queueIndex = 0;}
		}


		err |= addMessageToQueues_blocking((uint8_t *)(bufferPtr16Bit), (((uint32_t)accelQueueCopy.size) * 2 * 3));
		free(bufferPtr16Bit);

// Camera

		err |= addMessageToQueues_blocking(      (uint8_t *)&(cameraQueueCopy.elements[cameraQueueCopy.head]), (cameraQueueCopy.size - cameraQueueCopy.head) * (sizeof(struct STR_Jack_CameraTarget)));
		if(err != NO_ERROR){	return msgLenght;}
		if(cameraQueueCopy.head != 0){

			err |= addMessageToQueues_blocking((uint8_t *)&(cameraQueueCopy.elements[0]), (cameraQueueCopy.head) * (sizeof(struct STR_Jack_CameraTarget)));

		}


		if(err == NO_ERROR)
		{
//							xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//							PRINTF("Logging task: Sent Data.  \r\n");
//							xSemaphoreGive(xPrintMutex);
			sent_frames++;
		}

	}else if(buffer[sizeof(loggingMessage) + 4 - 1] == 0x02){
		// Camera Data.
		msgLenght = 44;
#if(1)
//						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//						PRINTF("Logging task: Sending Images... \r\n");
//						xSemaphoreGive(xPrintMutex);
		memset(&loggingMessage, 0, sizeof(loggingMessage));
		loggingMessage.STX = 0x000034E0;

		loggingMessage.LEN = sizeof(loggingMessage) - (sizeof(loggingMessage.STX) + sizeof(loggingMessage.LEN));

		loggingMessage.TYPE = 0x00000003;
		loggingMessage.VER = 0x00010001;
		loggingMessage.NODE = node_num;
		loggingMessage.TIME = loggingMessageSequence++;		// TODO.  I know this isn't Time.  But it's sequence... and that's the best we're going to do for now.
		loggingMessage.SUB_MSG_TYPE = 0x00000002;
		err = NO_ERROR;

		xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
		memcpy(&videoQueueCopy, &videoQueue, (sizeof(videoQueueCopy)));
		xSemaphoreGive( xLoggingMutex);
// calculate length
		for(i = 0; i < videoQueueCopy.size; i++){
			loggingMessage.LEN += (videoQueueCopy.elements[i].imageSize);
		}
// write the headder
//						err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(loggingMessage), (sizeof(loggingMessage)));
		err |= addMessageToQueues_blocking(		 (uint8_t *)&(loggingMessage), (sizeof(loggingMessage)));
		if(err != NO_ERROR){return msgLenght;}

// write the elements (in order)
		i = videoQueueCopy.head;
		if(i >= videoQueueCopy.size) { i = 0;}

		uint8_t *msgBufferPtr;
		uint32_t imageSize;

		do{


			i++;
			if(i >= videoQueueCopy.size){ i = 0;}

			msgBufferPtr = (uint8_t *)&(videoQueueCopy.elements[i]);
			memcpy(&imageSize, msgBufferPtr, 4);	// image Size is first 4 bytes.
//							xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//								PRINTF("Image %i, %i... \r\n", i, imageSize);
//							xSemaphoreGive(xPrintMutex);

			msgBufferPtr += 4;
// image size (4 bytes)
			if(imageSize > sizeof(videoQueueCopy.elements)){
			//	i = i + 1;
			//	if(i >= videoQueueCopy.size) { i = 0;}
				continue;
			}
			if(imageSize <= 0){
			//	i = i + 1;
			//	if(i >= videoQueueCopy.size) { i = 0;}
				continue;
			}
			err |= addMessageToQueues_blocking((uint8_t *) &imageSize, sizeof(imageSize));
// remainder of image (in 1000 byte blocks)
//			for(j = 0; j < imageSize; j+=1000){
//				length = imageSize - j;
//				if(length > 1000){
//					length = 1000;
//				}

				err |= addMessageToQueues_blocking(	(msgBufferPtr), imageSize);
//				if(err != NO_ERROR){	break;}
//			//	vTaskDelay(5);
//			}
			if(err != NO_ERROR){
				break;}
		}while(videoQueueCopy.head != i);

		if(err == NO_ERROR){
//							xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//							PRINTF("Logging task: Sent Images. total size: %u \r\n", loggingMessage.LEN);
//							xSemaphoreGive(xPrintMutex);
		}

	free(msgBufferPtr);
#endif

	}else if(buffer[sizeof(loggingMessage) + 4 - 1] == 0x03){
		// Audio Data.		// msgLength = 44

		msgLenght = 44;
#if(0)
//						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//						PRINTF("Logging task: Sending Audio... \r\n");
//						xSemaphoreGive(xPrintMutex);
		memset(&loggingMessage, 0, sizeof(loggingMessage));
		loggingMessage.STX = 0x000034E0;

		loggingMessage.LEN = sizeof(loggingMessage) - (sizeof(loggingMessage.STX) + sizeof(loggingMessage.LEN));

		loggingMessage.TYPE = 0x00000003;
		loggingMessage.VER = 0x00010001;
		loggingMessage.NODE = node_num;
		loggingMessage.TIME = loggingMessageSequence++;		// TODO.  I know this isn't Time.  But it's sequence... and that's the best we're going to do for now.
		loggingMessage.SUB_MSG_TYPE = 0x00000003;
		err = NO_ERROR;

		xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
		memcpy(&audioQueueCopy, &audioQueue, (sizeof(audioQueueCopy)));
		xSemaphoreGive( xLoggingMutex);

		// calculate length


		loggingMessage.LEN += audioQueueCopy.size * NUM_MICS * (sizeof(uint16_t)) ;
		loggingMessage.LEN += sizeof(i);
		loggingMessage.LEN += sizeof(wavHeadder);


//						err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(loggingMessage), (sizeof(loggingMessage)));
		err |= addMessageToQueues_blocking(		 (uint8_t *)&(loggingMessage), (sizeof(loggingMessage)));
		if(err != NO_ERROR){	return msgLenght;	}

		i = sizeof(audioQueueCopy.elements);

//						err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(i), (sizeof(i)));
		err |= addMessageToQueues_blocking(		 (uint8_t *)&(i), (sizeof(i)));
		if(err != NO_ERROR){	return msgLenght;}

//						err |= writeMsg_withTimeout_socket(sock, (uint8_t *)&(wavHeadder), (sizeof(wavHeadder)));
		err |= addMessageToQueues_blocking(		 (uint8_t *)&(wavHeadder), (sizeof(wavHeadder)));
		if(err != NO_ERROR){	return msgLenght;}

// write the elements (in order)
			// write [head] to the end.



		i = 0;
		int byteCounter = 0;
		int totalByteCounter = 0;
		uint16_t *wavFileAudioBuffer;
		wavFileAudioBuffer = malloc(1000 * 2 * 3);
		length = (audioQueueCopy.size - audioQueueCopy.head) * (sizeof(uint16_t)) * NUM_MICS;

		if(wavFileAudioBuffer  != NULL){


			for(j = audioQueueCopy.head + 1; j != audioQueueCopy.head; j += 1000){
				if(j > audioQueueCopy.size){j = 0;}
				for(i = 0; i < 1000; i++){
					if((j+i) >= audioQueueCopy.size){break;}
					wavFileAudioBuffer[3 * i    ] = audioQueueCopy.elements[j+i][0];
					wavFileAudioBuffer[3 * i + 1] = audioQueueCopy.elements[j+i][1];
					wavFileAudioBuffer[3 * i + 2] = audioQueueCopy.elements[j+i][2];
					byteCounter += 6;
					if((j+i) == audioQueueCopy.head){break;}
				}
				err |= addMessageToQueues_blocking(		 (uint8_t *)(wavFileAudioBuffer), byteCounter );
				totalByteCounter += byteCounter ;
				byteCounter = 0;
				if(err != NO_ERROR){	break;}
				if((j+i) == audioQueueCopy.head){break;}
			}
			free(wavFileAudioBuffer);
		}
		if(err != NO_ERROR){	return msgLenght;}


#endif

	}else{
		// we're here if we didn't understand the msg.  So, we'll end up returning the same nubmer of
		// bytes we were sent.  Thus emptying the buffer.

	}

return msgLenght;




}


#endif
