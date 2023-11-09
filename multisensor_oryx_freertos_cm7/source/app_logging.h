

#ifndef _APP_LOGGING_H
#define _APP_LOGGING_H

#include "app_video.h"
#include "JPEGENC.h"
#include "app_jack.h"
#ifndef float32_t
	#define float32_t float
#endif
#ifndef float64_t
	#define float64_t double
#endif

extern SemaphoreHandle_t xLoggingMutex;

#pragma pack(1)
struct STR_LOGGING_RadarElement{
	int8_t		angle;
	uint8_t		distance;
	uint8_t		magnitude;
};

#pragma pack(1)
struct STR_AudioQueue{
	uint32_t head;
	uint32_t tail;
	uint32_t size;
	uint16_t elements[320000][3];	// save as A,B,C,A,B,C...
};
#pragma pack()

struct STR_LOGGING_VideoElement{
	uint32_t imageSize;
	uint8_t image[JPG_IMAGE_BUFFER_SIZE];
};


#pragma pack(1)
// fuc!k you packing.  This should all be on 32 bit boundaries, but without the pack PRAGMA, this evaluates to 32 bytes... not 28 as it should.
struct STR_LOGGING_MSG_Headder {
	uint32_t	STX;
	uint32_t	LEN;
	uint32_t	TYPE;
	uint32_t	VER;
	uint32_t	NODE;
	uint64_t	TIME;
	uint32_t	SUB_MSG_TYPE;
//	struct STR_LOGGING_RadarMsg radarData[]


} ;
#pragma pack()









void tcp_logging_task(void *pvParameters);
void logging_task(void *pvParameters);
void enqueueVideoElement(JPEGIMAGE *jpgImage, uint32_t imageSize, uint32_t numbElements);
void enqueueRadarElement(int16_t *angle, int16_t *distance, int16_t *magnitude, uint16_t numbElements);
void enqueueCameraElement(struct STR_Jack_CameraTarget *cameraTargetList, uint16_t numbElements);
void enqueuePIRElement(uint8_t *pir0, uint8_t *pir1, int numbElements);
void enqueueAudioElement(int16_t *ch0, int16_t *ch1, int16_t *ch2, int numbElements);
void enqueueAccelElement(int16_t *x, int16_t *y, int16_t *z, int numbElements);
void getVideoElement(uint32_t *jpgSize, uint8_t *jpg);
uint8_t *getVideoElementPtr(uint32_t *jpgSize);



#endif
