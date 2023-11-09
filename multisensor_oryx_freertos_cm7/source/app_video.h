/*
 * app_video.h
 *
 *  Created on: Jul. 7, 2022
 *      Author: stippett
 */

#ifndef APP_VIDEO_H_
#define APP_VIDEO_H_


#include "app_shared.h"
#include "app_video.h"
#include "JPEGENC.h"
/*******************************************************************************
 * Config Definitions
 ******************************************************************************/

	// as of 2022 09 14, ucQFactor BEST = 28KBytes, HIGH = 18KBytes, MED = 11KBytes (all are approx and vary somewhat).
#define JPG_IMAGE_QUALITY	  		JPEG_Q_LOW
#define JPG_IMAGE_BUFFER_SIZE 		24576


#define CP437_Period	10
#define CP437_Collon	11
#define CP437_N			12
#define CP437_Space		13
#define CP437_Asterisk	14
#define CP437_DebugBoxLow			15
#define CP437_DebugBoxHigh 			16
#define CP437_DebugBoxHighAndLow	17


#define DECORATION_MASK_TL_CHARACTER_COUNT		24		// top left
#define DECORATION_MASK_TR_CHARACTER_COUNT		11		// top Right
#define CHARACTER_HEIGHT					14
#define CHARACTER_WIDTH						9

#define ACTIVE_TARGET_LIST_SIZE		5

// character mask can be up to 24 characters. 9 pixels across x 14 pixels high for each character.
extern uint8_t decorationMask_TL[CHARACTER_HEIGHT][(DECORATION_MASK_TL_CHARACTER_COUNT*CHARACTER_WIDTH)];
extern uint8_t decorationMask_TR[CHARACTER_HEIGHT][(DECORATION_MASK_TR_CHARACTER_COUNT*CHARACTER_WIDTH)];

extern void generateDecorationMask(char *text, uint32_t length, uint32_t maskID);


extern const uint8_t codePage437[18][CHARACTER_HEIGHT][CHARACTER_WIDTH];

/*******************************************************************************
 * Definitions
 ******************************************************************************/



extern SemaphoreHandle_t xVideoMutex;

extern uint8_t video_error;
extern uint8_t video_error_reported;

void tcp_video_task(void *pvParameters);
void video_task(void *pvParameters);
uint32_t getCameraMagnitude(void);
int32_t	getCameraError(void);
int getKFilteredVideoTarget(int index, int *x, int *y, int *radarValidated, int *h, int *w);
uint32_t getLatestJpeg(uint8_t *jpgData);
void setLiveSettingsVideo(uint32_t decorations, uint32_t stageSelect, uint8_t useLiveSettings);

#endif
