/*
 * app_video.c
 *
 *  Created on: Jul. 7, 2022
 *      Author: stippett
 */


#include <string.h>
#include <math.h>

#include "cr_section_macros.h"

#include "app_audio.h"


//#include "app_shield.h"

//#include "McuWait.h"
//#include "McuRTOS.h"

/* lwIP */
#include "lwip/opt.h"
#include "lwip/netifapi.h"
#include "lwip/tcpip.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/timeouts.h"

/* Video */
#include "camera_support.h"
#include "fsl_video_common.h"
#include "fsl_camera.h"
#include "fsl_camera_device.h"
#include "fsl_ov5640.h"
#include <assert.h>

#include "board.h"
#include "app_video.h"
#include "app_jack.h"

#include "app_shared.h"
#include "JPEGENC.h"
#include "app_logging.h"
#include "fsl_cache.h"
#include "ov5640.h"
#include "app_config.h"
#include "app_board.h"
#include "app_microwave.h"
#include "fsl_csi_camera_adapter.h"
#include "sod.h"
/* Camera and Video */
/* CSI output frame buffer is XRGB8888. */
#define APP_CAMERA_BUFFER_BPP 4

#define APP_CAMERA_BUFFER_COUNT 3

#define MAX_VIDEO_TARGETS 			(10)		// CHanged this from 20 to 10 at GSX show.
#define VIDEO_TARGET_TIMEOUT		(20)		// this is subtracted every frame.  So it's in units of FrameRate.

#define JPG_QUALITYCHANGE_DEBOUNCE_VAL			20
#define JPG_QUALITYCHANGE_INCREASE_THRESHOLD 	(10 * 1024)	// the size, in bytes, of a low quality image which is
													// considered small enough to increase the quality back to medium

// the jpgTempImageBuffer makes a copy of the image.  We use it because the image is held in non-cacheable memory (terribly slow access speeds).
__NOINIT(SRAM_OC1) uint8_t jpgTempImageBuffer[(DEMO_CAMERA_HEIGHT * DEMO_CAMERA_WIDTH * APP_CAMERA_BUFFER_BPP)];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
int backgroundDeletion(uint8_t *imageOut, uint8_t *image);
int blobMotionDetection(uint8_t *image);
int encodeJPG(JPEGIMAGE *jpg, uint8_t *image, uint8_t encodeQuality);
void serviceVideoTargetList(void);
void addVideoTarget(sod_box *box);
void flushVideoTargets(void);
uint32_t getBoxFromVideoTarget(uint32_t index, sod_box *targetBox);
uint32_t kalmanVideo_MultiTarget_TargetListLookup(float newX, float newY);
int getKFilteredVideoTarget(int index, int *x, int *y, int *radarValidated, int *h, int *w);
/*******************************************************************************
 * Variables
 ******************************************************************************/
//__NOINIT(RAM3) static uint8_t s_cameraBuffer[APP_CAMERA_BUFFER_COUNT][DEMO_CAMERA_HEIGHT][DEMO_CAMERA_WIDTH * APP_CAMERA_BUFFER_BPP];
//__NOINIT(RAM6) static uint8_t s_cameraBuffer[1][DEMO_CAMERA_HEIGHT][1 * APP_CAMERA_BUFFER_BPP];
//static uint8_t s_testBuffer[DEMO_CAMERA_HEIGHT][DEMO_CAMERA_WIDTH * APP_CAMERA_BUFFER_BPP];


__BSS(BOARD_SDRAM) static uint8_t s_cameraBuffer[APP_CAMERA_BUFFER_COUNT][DEMO_CAMERA_HEIGHT][DEMO_CAMERA_WIDTH * APP_CAMERA_BUFFER_BPP];
//AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_cameraBuffer[APP_CAMERA_BUFFER_COUNT][DEMO_CAMERA_HEIGHT]
//                                                           [DEMO_CAMERA_WIDTH * APP_CAMERA_BUFFER_BPP],
//                              DEMO_CAMERA_BUFFER_ALIGN);


__NOINIT(SRAM_OC1)	float imageInGreyScale[120 * 160];
__NOINIT(SRAM_OC1)	float binarizedImage[120 * 160];			//
//__NOINIT(SRAM_OC2)	float erodedImage[120 * 160];			//
__NOINIT(SRAM_OC2)	float backgroundImage[120 * 160][3];		// RGB background image.
__NOINIT(SRAM_OC2)	float varianceImage[120 * 160];
__NOINIT(SRAM_OC2)	float dilatedCopyImage[120 * 160];			//
__NOINIT(SRAM_OC2)	float workingCopyImage[120 * 160];			//


int32_t	cameraErrorFlag = 0;
int32_t	cameraErrorFlagPrev = 0;

uint8_t video_error = 0;
uint8_t video_error_reported = 0;

float videoCartesianToActualCartesainFactor = 1;

int32_t	getCameraError(void){
	int32_t err;
	err = cameraErrorFlag;
	if (cameraErrorFlag != 0) {
		return 1;
	} // the above is temp

	return err;
}

void camera_callback(camera_receiver_handle_t *handle, status_t status, void *userData)
{
//	DIAG_E8_ON;
//	LED_ON_3;
//	LED_OFF_3;
}


struct S_kalmanFilter videoKalmanVariables[ACTIVE_TARGET_LIST_SIZE][2];
#pragma pack(0)


uint8_t liveSettingsEnable = 0; // controls if config or live settings are used. Set or cleared by setLiveSettingsVideo
// used by the getDecorationEnable function and the getOutputStageSelect function
struct STR_cameraConfig liveSettings;


#define VIDEO_DECORATION_IP 			1
#define VIDEO_DECORATION_MAC 			2
#define VIDEO_DECORATION_TIME			3
#define VIDEO_DECORATION_NODE			4
#define VIDEO_DECORATION_TRACKINGBOXES	5
#define VIDEO_DECORATION_TRACKING		6

uint8_t getDecorationEnable(uint8_t decorationType){
	if(decorationType == VIDEO_DECORATION_IP){
		if (liveSettingsEnable){
			return (liveSettings.decorationIPAddress != 0);
		}
		else{
			return (configParam.cameraConfig.decorationIPAddress != 0);
		}
	}
	if(decorationType == VIDEO_DECORATION_MAC){
		if (liveSettingsEnable){
			return (liveSettings.decorationMACAddress != 0);
		}
		else{
			return (configParam.cameraConfig.decorationMACAddress != 0);
		}
	}
	if(decorationType == VIDEO_DECORATION_TIME){
		if (liveSettingsEnable){
			return (liveSettings.decorationTimeOfDay != 0);
		}
		else{
			return (configParam.cameraConfig.decorationTimeOfDay != 0);
		}
	}
	if(decorationType == VIDEO_DECORATION_NODE){
		if (liveSettingsEnable){
			return (liveSettings.decorationNodeAddress != 0);
		}
		else{
			return (configParam.cameraConfig.decorationNodeAddress != 0);
		}
	}
	if(decorationType == VIDEO_DECORATION_TRACKINGBOXES){
		if (liveSettingsEnable){
			return (liveSettings.decorationTrackingBoxes != 0);
		}
		else{
			return (configParam.cameraConfig.decorationTrackingBoxes != 0);
		}
	}
	if(decorationType == VIDEO_DECORATION_TRACKING){
		if (liveSettingsEnable){
			return (liveSettings.decorationTracking != 0);
		}
		else{
			return (configParam.cameraConfig.decorationTracking != 0);
		}
	}
	return 0;
}
void setLiveSettingsVideo(uint32_t decorations, uint32_t stageSelect, uint8_t useLiveSettings){
	if(useLiveSettings){


		// copy new settings over to the live settings struct
		liveSettings.decorationIPAddress = decorations & JACK_LIVESETTINGS_VIDDECOR_IPADDR;
		liveSettings.decorationMACAddress = configParam.cameraConfig.decorationMACAddress; // no live setting for MAC address yet
		liveSettings.decorationNodeAddress = decorations & JACK_LIVESETTINGS_VIDDECOR_NODE;
		liveSettings.decorationTimeOfDay = decorations & JACK_LIVESETTINGS_VIDDECOR_TIME;
		liveSettings.decorationTrackingBoxes = decorations & JACK_LIVESETTINGS_VIDDECOR_TRACKINGBOX;
		liveSettings.decorationTracking = decorations & JACK_LIVESETTINGS_VIDDECOR_TRACKING;

		liveSettings.cameraImageSelection = (uint8_t) stageSelect;

		liveSettingsEnable = 1; // enable live settings after setup in case of task switch
	}
	else{
		// go back to using the device config
		liveSettingsEnable = 0;
	}

	// re-generate our top left decoration mask.
	char maskText[22];
	if(		    (getDecorationEnable(VIDEO_DECORATION_NODE) == TRUE) &&  (getDecorationEnable(VIDEO_DECORATION_IP) == TRUE)){
		sprintf(maskText, "N: %u %u.%u.%u.%u", configParam.nodeAddress, configParam.ipAddress[0], configParam.ipAddress[1], configParam.ipAddress[2], configParam.ipAddress[3]);
	} else if(	(getDecorationEnable(VIDEO_DECORATION_NODE) == FALSE) && (getDecorationEnable(VIDEO_DECORATION_IP) == TRUE)){
		sprintf(maskText, "      %u.%u.%u.%u", configParam.ipAddress[0], configParam.ipAddress[1], configParam.ipAddress[2], configParam.ipAddress[3]);
	} else if(	(getDecorationEnable(VIDEO_DECORATION_NODE) == TRUE) &&  (getDecorationEnable(VIDEO_DECORATION_IP) == FALSE)){
		sprintf(maskText, "N: %u", configParam.nodeAddress);
	}else{
		sprintf(maskText, "                 ");
	}
	generateDecorationMask(maskText, strlen(maskText), 0);

}
uint8_t getOutputStageSelect(){
	if (liveSettingsEnable){
		return liveSettings.cameraImageSelection;
	}
	else{
		return configParam.cameraConfig.cameraImageSelection;
	}
}

/* The callback function. */




SemaphoreHandle_t xVideoMutex;
TimerHandle_t xTimerHandle_video;
int imageAvailableFlag = 0;
int sendVideoFlag = 0;
int getImageFlag = 0;
int imagePrescaller = 0;
void vTimerCallBack_video( TimerHandle_t xTimer )
{	// this is a 100mS timer.
	if(imagePrescaller >= 10){
		imagePrescaller	= 0;
	}

	if((imagePrescaller % 2) == 0){
		sendVideoFlag = 1;
	}

	if((imagePrescaller % 2) == 0 ){
		getImageFlag = 1;
	}
	imagePrescaller++;
}

void APP_InitCamera(void)
{
	camera_config_t cameraConfig;

	memset(&cameraConfig, 0, sizeof(cameraConfig));

	BOARD_InitCameraResource();

	/* CSI input data bus is 24-bit, and save as XRGB8888.. */
	cameraConfig.pixelFormat                = kVIDEO_PixelFormatXRGB8888;
	cameraConfig.bytesPerPixel              = APP_CAMERA_BUFFER_BPP;
	cameraConfig.resolution                 = FSL_VIDEO_RESOLUTION(DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT);
	cameraConfig.frameBufferLinePitch_Bytes = DEMO_CAMERA_WIDTH * APP_CAMERA_BUFFER_BPP;
	cameraConfig.interface                  = kCAMERA_InterfaceGatedClock;
	cameraConfig.controlFlags               = DEMO_CAMERA_CONTROL_FLAGS;
	cameraConfig.framePerSec                = DEMO_CAMERA_FRAME_RATE;

//t.s.	CAMERA_RECEIVER_Init(&cameraReceiver, &cameraConfig, NULL, NULL);
	CAMERA_RECEIVER_Init(&cameraReceiver, &cameraConfig, &camera_callback, NULL);

	BOARD_InitMipiCsi();

	cameraConfig.pixelFormat   = kVIDEO_PixelFormatRGB565;
	cameraConfig.bytesPerPixel = 2;
	cameraConfig.resolution    = FSL_VIDEO_RESOLUTION(DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT);
	cameraConfig.interface     = kCAMERA_InterfaceMIPI;
	cameraConfig.controlFlags  = DEMO_CAMERA_CONTROL_FLAGS;
	cameraConfig.framePerSec   = DEMO_CAMERA_FRAME_RATE;
	cameraConfig.csiLanes      = DEMO_CAMERA_MIPI_CSI_LANE;
	CAMERA_DEVICE_Init(&cameraDevice, &cameraConfig);

	//CAMERA_DEVICE_Start(&cameraDevice);

	/* Submit the empty frame buffers to buffer queue. */
	for (uint32_t i = 0; i < APP_CAMERA_BUFFER_COUNT; i++)
	{
		CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(s_cameraBuffer[i]));
	}
}

#define HEADER_SIZE 13
uint8_t	video_sequence_num;


union CAMERA_DATA_FRAMING{
	uint32_t *uint32Ptr;
	uint8_t *uint8Ptr;
};




#define OV5640_ROLLING_SHUTTER_ON		OV5640_WriteReg((&cameraDevice), 0x3817, 0b00000000);		// 0x3817[3] is to turn off rolling shutter mode.  We will only capture data on Frex Requests.
#define OV5640_ROLLING_SHUTTER_OFF		OV5640_WriteReg((&cameraDevice), 0x3817, 0b00001000);		// 0x3817[3] is to turn off rolling shutter mode.  We will only capture data on Frex Requests.

#define OV5640_STROBE_PULSE__0_15(strobePulseSetting)			OV5640_WriteReg((&cameraDevice), 0x3B06, ((strobePulseSetting) & 0x0F));				// Flash Strobe Pulse Width is ( 3b06[3:0] -1 ) Tlines
#define	OV5640_FREX_REQUEST										OV5640_WriteReg((&cameraDevice), 0x3B08, 0x01);		// FREX request.  When we're not in 'rolling / normal mode' we need a FREX request to initiate a camera sample.

#define OV5640_PAD_OUT_0_REG		(0x3016)
#define OV5640_PAD_OUT_1_REG		(0x3017)

#define OV5640_PAD_OUT_0__STROBE	(0b00000010)
#define OV5640_PAD_OUT_1__FREX		(0b10000000)
#define OV5640_PAD_OUT_1__VSYNC		(0b01000000)
#define OV5640_PAD_OUT_1__HREF		(0b00100000)
#define OV5640_PAD_OUT_1__PCLK		(0b00010000)


SemaphoreHandle_t xJPEGVideoMutex;	//	this simply protects dual access to JPG
SemaphoreHandle_t xVideoTargetMutex;	// this protects dual access to the video target list.
JPEGIMAGE jpg;
//uint8_t *imagePingPongBuffer[2]; // holds the actual image data
uint8_t imageInBufferPtr;	// pointer to one of the above buffers
uint8_t imageOutBuffer[320*240*4]; // pointer to the other buffer

uint8_t currentImage[320*240*4];
struct STR_cameraConfig	cameraConfig;

void setCameraConfig(struct STR_cameraConfig *newConfig){
	memcpy(&cameraConfig, newConfig, sizeof(newConfig));
}
void getCameraConfig(struct STR_cameraConfig *newConfig){
	memcpy(newConfig, &cameraConfig, sizeof(newConfig));
}
uint32_t cameraMagnitude = 0;
uint32_t getCameraMagnitude(void){
	uint32_t rv;
	xSemaphoreTake( xJPEGVideoMutex, ( TickType_t ) portMAX_DELAY );
		rv = cameraMagnitude;
		cameraMagnitude = 0;
	xSemaphoreGive( xJPEGVideoMutex);
	return rv;
}


void video_task(void *pvParameters)
{


//		uint8_t header[HEADER_SIZE];
	//TaskHandle_t video_polling_handle = NULL;

	LED_ARRAY_CHARGE_OFF;
	int retryCount;
	int32_t i;
//	uint8_t exposureValueLSB = 0, exposureValuemSB = 0, exposureValueMSB = 0;
//	uint8_t buffer[4];
//	int exposureFlashThreshold = 0xC00000;
//	int nightMode = 0;
	err_t err;
	int exposureValue = 0;
	int exposureFilter = 55;
	enum LIGHTING_STATE {lightingState_DAY = 0, lightingState_NIGHT = 1, lightingState_NIGHT_LIGHT = 2} lightingState, lightingStatePrevious;




	struct v4l2_mbus_framefmt mf;






	char maskText[22];
	if(		    (getDecorationEnable(VIDEO_DECORATION_NODE) == TRUE) &&  (getDecorationEnable(VIDEO_DECORATION_IP) == TRUE)){
		sprintf(maskText, "N: %u %u.%u.%u.%u", configParam.nodeAddress, configParam.ipAddress[0], configParam.ipAddress[1], configParam.ipAddress[2], configParam.ipAddress[3]);
	} else if(	(getDecorationEnable(VIDEO_DECORATION_NODE) == FALSE) && (getDecorationEnable(VIDEO_DECORATION_IP) == TRUE)){
		sprintf(maskText, "      %u.%u.%u.%u", configParam.ipAddress[0], configParam.ipAddress[1], configParam.ipAddress[2], configParam.ipAddress[3]);
	} else if(	(getDecorationEnable(VIDEO_DECORATION_NODE) == TRUE) &&  (getDecorationEnable(VIDEO_DECORATION_IP) == FALSE)){
		sprintf(maskText, "N: %u", configParam.nodeAddress);
	}else{
		sprintf(maskText, "                 ");
	}
	generateDecorationMask(maskText, strlen(maskText), 0);


	memset(backgroundImage, 0, sizeof(backgroundImage));

	union CAMERA_DATA_FRAMING cameraReceivedFrameAddr;
	do{
		cameraReceivedFrameAddr.uint32Ptr = NULL;

		CAMERA_RESET_ASSERT;				// Camera RESETB - put in to RESET

		vTaskDelay(100);

		CAMERA_RESET_UNASSERT;				// Camera RESETB - put out of RESET

		BOARD_Camera_I2C_Init();

		vTaskDelay(50);						//	We need 20ms after reset goes high until we send I2C commands



		cameraErrorFlagPrev = cameraErrorFlag;
		cameraErrorFlag = ov5640_probe();
		if (cameraErrorFlag != cameraErrorFlagPrev){
			// change in error state
			// so flag to the jack task that there's a change
			diagnostic_change_flag |= 1;
			if(cameraErrorFlag != 0){
				sys_stats.sensorstat.faultCamera_count++;
			}
		}

		flushVideoTargets();
		for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i++){
			kFilter_resetTarget(&(videoKalmanVariables[i][0]));
			kFilter_resetTarget(&(videoKalmanVariables[i][1]));

		}


			exposureValue = getOV5640_AVG_READOUT();

			mf.code = V4L2_MBUS_FMT_YUYV8_2X8;
			mf.width = DEMO_CAMERA_WIDTH;
			mf.height = DEMO_CAMERA_HEIGHT;
			mf.colorspace = V4L2_COLORSPACE_JPEG;
			ov5640_s_fmt(&mf);
			ov5640_config_preview();
			ov5640_config_capture();
	//		vTaskDelay(1000);
			ov5640_s_stream(1);
			//			runmode = CAM_RUNNING_MODE_CAPTURE;
			ov5640_reg_writes(ov5640_stream);
			OV5640_WriteReg((&cameraDevice), 0x4202, 0x00);

			exposureValue = getOV5640_AVG_READOUT();
			exposureFilter = exposureValue;
			lightingState = lightingState_DAY;
			lightingStatePrevious = lightingState_DAY;
			if(lightingStatePrevious != lightingState_DAY){
				setNightMode(0);
			}


			APP_InitCamera();
			cameraReceivedFrameAddr.uint8Ptr = (uint8_t *) &(s_cameraBuffer[0][0]);
			CAMERA_DEVICE_Start(&cameraDevice);
			CAMERA_RECEIVER_Start(&cameraReceiver);


			err = ERR_OK;
			LED_ARRAY_STROBE_OFF;
			LED_ARRAY_CHARGE_ON;
			do {


				if(getImageFlag){

					getImageFlag = 0;

					if(configParam.ir_LEDs_Enable == 1){
						if(lightingState == lightingState_NIGHT_LIGHT)
						{
							LED_ARRAY_STROBE_ON;
							LED_ARRAY_CHARGE_ON;
						}else{
							LED_ARRAY_CHARGE_OFF;
						}
					}
					else{
						LED_ARRAY_CHARGE_OFF;
					}



					retryCount = 500;
					cameraReceivedFrameAddr.uint32Ptr = NULL;
					do{
						if(retryCount-- <= 0) break;
						vTaskDelay(10);
						L1CACHE_InvalidateDCacheByRange((uint32_t)&cameraReceiver, (sizeof(cameraReceiver)));
						i = CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, (uint32_t *) &(cameraReceivedFrameAddr.uint32Ptr));

					}while (i != kStatus_Success);


					if(cameraReceivedFrameAddr.uint32Ptr != NULL){
						L1CACHE_InvalidateDCacheByRange((uint32_t)cameraReceivedFrameAddr.uint32Ptr, (DEMO_CAMERA_HEIGHT * DEMO_CAMERA_WIDTH * APP_CAMERA_BUFFER_BPP));
						memcpy(currentImage, cameraReceivedFrameAddr.uint32Ptr, sizeof(currentImage));
						CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(cameraReceivedFrameAddr.uint32Ptr )  );
					}else{
						for (i = 0; i < APP_CAMERA_BUFFER_COUNT; i++)
						{
							CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(s_cameraBuffer[i]));
						}
						continue;
					}
//					CSI_DisableInterrupts(((csi_resource_t *)cameraReceiver.resource)->csiBase, (uint32_t)kCSI_RxBuffer1DmaDoneInterruptEnable | (uint32_t)kCSI_RxBuffer0DmaDoneInterruptEnable);
//				    CSI_EnableInterrupts(((csi_resource_t *)cameraReceiver.resource)->csiBase, (uint32_t)kCSI_RxBuffer1DmaDoneInterruptEnable | (uint32_t)kCSI_RxBuffer0DmaDoneInterruptEnable);

					exposureValue = getOV5640_AVG_READOUT();
					exposureFilter = (exposureFilter * 9 + exposureValue) / 10;

					switch(lightingState){
					case lightingState_DAY:
						if(exposureFilter < 20){
							lightingState = lightingState_NIGHT;
						}
						if(lightingStatePrevious != lightingState_DAY){
							setNightMode(0);
						}
						break;
					case lightingState_NIGHT:
						if(exposureFilter > 40){
							lightingState = lightingState_DAY;
						}
						if(exposureFilter < 20){
							lightingState = lightingState_NIGHT_LIGHT;
						}
						if(lightingStatePrevious != lightingState_NIGHT){
							setNightMode(1);
						}
						break;
					case lightingState_NIGHT_LIGHT:
						if(exposureFilter > 40){
							lightingState = lightingState_NIGHT;
						}
						if(lightingStatePrevious != lightingState_NIGHT_LIGHT){
							setNightMode(1);
						}
						break;

					}
					lightingStatePrevious = lightingState;

					LED_ARRAY_STROBE_OFF;

					if(retryCount > 0) {


						blobMotionDetection((uint8_t *)(currentImage));
					//	blobMotionDetection((uint8_t *)(cameraReceivedFrameAddr.uint32Ptr));

						if(sendVideoFlag){
							sendVideoFlag = 0;
							xSemaphoreTake( xJPEGVideoMutex, ( TickType_t ) portMAX_DELAY );
								memcpy(jpgTempImageBuffer, currentImage, sizeof(jpgTempImageBuffer));
								imageAvailableFlag = 1;
							xSemaphoreGive( xJPEGVideoMutex );

						}

					}else{
						err = -1;
					}





					serviceVideoTargetList();


				}
				else{
					cameraReceivedFrameAddr.uint32Ptr = NULL;
					retryCount = 100;
					do{
						if(retryCount-- <= 0) break;
						vTaskDelay(10);
						L1CACHE_InvalidateDCacheByRange((uint32_t)&cameraReceiver, (sizeof(cameraReceiver)));
						i = CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, (uint32_t *) &(cameraReceivedFrameAddr.uint32Ptr));

					}while (i != kStatus_Success);


					if(cameraReceivedFrameAddr.uint32Ptr != NULL){
						L1CACHE_InvalidateDCacheByRange((uint32_t)cameraReceivedFrameAddr.uint32Ptr, (DEMO_CAMERA_HEIGHT * DEMO_CAMERA_WIDTH * APP_CAMERA_BUFFER_BPP));
						memcpy(currentImage, cameraReceivedFrameAddr.uint32Ptr, sizeof(currentImage));
						CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(cameraReceivedFrameAddr.uint32Ptr )  );
					}else{
						for (i = 0; i < APP_CAMERA_BUFFER_COUNT; i++)
						{
							CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(s_cameraBuffer[i]));
						}
						continue;
					}

					exposureValue = getOV5640_AVG_READOUT();
					exposureFilter = (exposureFilter * 9 + exposureValue) / 10;

					if(retryCount <0){
						APP_InitCamera();
						CAMERA_DEVICE_Start(&cameraDevice);
						CAMERA_RECEIVER_Start(&cameraReceiver);
						//						err = -1;
					}

				}
				cameraErrorFlagPrev = cameraErrorFlag;
				cameraErrorFlag = err;
				if (cameraErrorFlag != cameraErrorFlagPrev){
					// change in error state
					// so flag to the jack task that there's a change
					diagnostic_change_flag |= 1;
					if(cameraErrorFlag != 0){
						sys_stats.sensorstat.faultCamera_count++;
					}
				}

			}while(err == ERR_OK);


	}while(1);

}












void tcp_video_task(void *pvParameters)
{

	struct netconn *conn, *newconn = NULL;
	err_t err;
	ip_addr_t client_address;
	uint16_t client_port;

	size_t bytesWritten = 0;
	size_t bytesToWrite = 0;
	size_t byteCounter = 0;
	size_t index;
	uint32_t connectionEstablished = 0;
	uint32_t sent_frames;

	int rc;
	uint8_t encodeQuality = JPEG_Q_MED;
	int32_t jpgQualityChangeDebounce = JPG_QUALITYCHANGE_DEBOUNCE_VAL;
	uint8_t jpgQualityJustReduced = 0;


	LWIP_UNUSED_ARG(pvParameters);
	/* Store the handle of the calling task. */
//	xTaskToNotify_video = xTaskGetCurrentTaskHandle();
	vTaskDelay(1000);						// The delay here is a shitty hack.  It's to make sure we're done setting up the PHY before we use the same I2C port to talk to the camera.

	/* Create a new connection identifier. */
	/* Bind connection to well known port number 8. */


	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, IP_ADDR_ANY, 10009);
	LWIP_ERROR("Video task: invalid conn", (conn != NULL), return;);
	/* Tell connection to go into listening mode. */
	netconn_listen(conn);
	/* Main loop. Get sensor data and send via TCP */
	/* Grab new connection. */
	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
	PRINTF("Video task: awaiting new connection %p\r\n", newconn);
	xSemaphoreGive( xPrintMutex );

	while (1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS );
		conn->flags |= NETCONN_FLAG_NON_BLOCKING;		// added this so that the accept command doesn't block.  T.S. 2023 08 30
		err = ERR_OK;
		err = netconn_accept(conn, &newconn);
		conn->flags &= ~(NETCONN_FLAG_NON_BLOCKING);		// Now I clear it because i'm not sure how netcon send commands behave with the non-blocking.

		if(err == ERR_WOULDBLOCK){
			// we would have blocked.  thus we don't have a connection.
			connectionEstablished = 0;
		}
		else if (err != ERR_OK) {
			//netconn_close(newconn);
			//netconn_delete(newconn);
			vTaskDelay(1000 / portTICK_PERIOD_MS );

			conn = netconn_new(NETCONN_TCP);
			netconn_bind(conn, IP_ADDR_ANY, 10009);
			LWIP_ERROR("Video task: invalid conn", (conn != NULL), return;);
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);
			/* Main loop. Get sensor data and send via TCP */
			/* Grab new connection. */
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Video task: connection Reset.  Waiting...\r\n");
			xSemaphoreGive( xPrintMutex );
			connectionEstablished = 0;
			continue;
		} else{

			/* Process the new connection. */
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Video task: accepted new connection\r\n");
			xSemaphoreGive( xPrintMutex );

			err = netconn_peer(newconn, &client_address, &client_port);
			newconn->send_timeout = 200;
			connectionEstablished = 1;
		}

//		if (err != ERR_OK) {
//			netconn_close(newconn);
//		//	netconn_delete(newconn);
//		//	vTaskDelay(1000 / portTICK_PERIOD_MS );
//			connectionEstablished = 0;
//			continue;
//		}

		do{
			xSemaphoreTake( xJPEGVideoMutex, ( TickType_t ) portMAX_DELAY );
			if(imageAvailableFlag){
				rc = encodeJPG(&jpg, NULL, encodeQuality);
				if(rc == JPEG_NO_BUFFER){
					// no buffer space
					// reduce quality and reset our debounce value, then encode
					encodeQuality = JPEG_Q_LOW;
					jpgQualityChangeDebounce = JPG_QUALITYCHANGE_DEBOUNCE_VAL;
					jpgQualityJustReduced = 1; // size threshold experimentation purposes

//								rc = encodeJPG(&jpg, (uint8_t *)currentImage, encodeQuality);
					rc = encodeJPG(&jpg, NULL, encodeQuality);
									}
				if(rc <0){
					// fail to encode for reasons other than no buffer space.
					rc = 0;
					memset(&jpg, 0, sizeof(jpg));
					err = -1;	// fail.  Re-init the camera;
				}
				else{
					// successful encode
					if (encodeQuality == JPEG_Q_LOW && rc < JPG_QUALITYCHANGE_INCREASE_THRESHOLD ){
						// this image is small enough to consider increasing the quality back to medium
						if(jpgQualityChangeDebounce > 0){
							jpgQualityChangeDebounce--;
							if(jpgQualityJustReduced){
								// for experimentation purposes, if we get here then the size threshold should be increased
								// put breakpoint here to experiment
								assert(1);
							}
						}else{
							encodeQuality = JPEG_Q_MED;
						}
					}
					else if (jpgQualityJustReduced){
						jpgQualityJustReduced = 0; // size threshold experimentation purposes.
					}
				}


				enqueueVideoElement(&jpg, rc ,1);				// add it to our logging queue
				if(rc > 0){
					bytesToWrite = rc;
				}

				imageAvailableFlag = 0;

				byteCounter = 0;
				bytesWritten = 0;
				if(connectionEstablished){
					do{
						index = (uint32_t)(jpg.iDataSize) - byteCounter;
						if(index > 61440) {
							index = 61440;
							break;
						}
						//jpg TX   start vlc --demux mjpeg tcp://192.168.0.170:10009
						err = netconn_write_partly(newconn, (void *)(jpg.pOutput + byteCounter), index, NETCONN_COPY, &bytesWritten);

						if(bytesWritten == 0){
							err |= -1;
							break;
						}
						byteCounter += bytesWritten;
						vTaskDelay(5 / portTICK_PERIOD_MS );
					}while(byteCounter < bytesToWrite);

					if(err == ERR_OK){
						sent_frames++;
					}
				}
				xSemaphoreGive( xJPEGVideoMutex );
			} else{
				xSemaphoreGive( xJPEGVideoMutex );
				vTaskDelay(10 / portTICK_PERIOD_MS );
			}
		}while(err == ERR_OK);
		if(connectionEstablished){
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Video task: Got EOF, looping. %u sent\r\n", sent_frames);
			xSemaphoreGive( xPrintMutex );
			/* Close connection and discard connection identifier. */
			netconn_close(newconn);
			netconn_delete(newconn);

			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Video task: awaiting new connection %p\r\n", newconn);
			xSemaphoreGive( xPrintMutex );
		}
	}
}


#define BINARIZE_THRESHOLD 	 			(configParam.reserved0f)		//0.03
#define VARIANCE_THRESHOLD 	 			(configParam.reserved1f)		//1.2
#define BACKGROUND_IMAGE_TIME_CONSTANT	(configParam.reserved2f)		//0.9
#define VARIANCE_IMAGE_TIME_CONSTANT	(configParam.reserved3f)		//0.95

#define IMAGE_EROSION_FACTOR			 (configParam.reserved0i)  // (2)
#define IMAGE_DILATE_FACTOR			 	 (configParam.reserved1i)  // (12)
#define MIN_TARGET_SIZE					 (configParam.reserved2i)  // (35)
#define MAX_TARGET_SIZE					 (configParam.reserved3i) // (300)
#define CAMERA_OUTPUT_SELECTION_FROM_BLOB_DETECT	 (getOutputStageSelect())



#define CAMERA_OUTPUT_SELECT__RAW_IMAGE							(0)
#define CAMERA_OUTPUT_SELECT__BACKGROUND_IMAGE_RED					(1)
#define CAMERA_OUTPUT_SELECT__AFTER_BACKGROUND_SUBTRACTION	(2)
#define CAMERA_OUTPUT_SELECT__AVERAGE_VARIANCE_IMAGE					(3)
#define CAMERA_OUTPUT_SELECT__AFTER_DIGITIZATION				(4)
#define CAMERA_OUTPUT_SELECT__AFTER_ERODE						(5)
#define CAMERA_OUTPUT_SELECT__AFTER_DILATE						(6)





static int filter_cb(int width, int height)
{
	/* A filter callback invoked by the blob routine each time
	 * a potential blob region is identified.
	 * We use the `width` and `height` parameters supplied
	 * to discard regions of non interest (i.e. too big or too small).
	 */
	if (((width > (MAX_TARGET_SIZE/2)) && (height > (MAX_TARGET_SIZE/2))) || (width < (MIN_TARGET_SIZE/2)) || (height < (MIN_TARGET_SIZE/2))) {		//	this is done on the 160 x 120 image.  so divide by 2.
		/* Ignore small or big boxes */
		return 0;
	}
	return 1; /* Region accepted */
}

// We need to tell the SOD that this is a BMP file.  Because BMP files closely look like or XRGB stream.
// So, we'll make the first 54 bytes equivalent to a BMP headder.  After that, data starts at byte 54.
//unsigned char tempFileData[320 * 240 * 4 + 54 + 1] = {0x42, 0x4D, 0x36, 0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00
//												, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x20, 0x00, 0x00, 0x00
//												, 0x00, 0x00, 0x00, 0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//												, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// the below marks the image as a 320 x (-240) dimension.  Otherwise the SOD function flip the image vertically.  That's dumb.  And I believe incorrect.
//unsigned char tempFileData[320 * 240 * 4 + 54 + 1] = {0x42, 0x4D, 0x36, 0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00
//												, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0x10, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x20, 0x00, 0x00, 0x00
//												, 0x00, 0x00, 0x00, 0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//												, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//





int sod_erode_image_to_output(sod_img im, sod_img out);
int sod_dilate_image_to_output(sod_img im, sod_img out);
void sod_image_draw_box_on_RGBX(uint8_t *RGBXin, int x1, int y1, int x2, int y2, int img_h, int img_w, int r, int g, int b);
void SOD160_120_Greyscale_to_rgbx320_240(sod_img *sod, uint8_t *rgbx);

int blobMotionDetection(uint8_t *image)
{


	sod_img imgInCopy;
	sod_img erodeImg;
	sod_img binImg;
	sod_img dilImg;
	sod_img workingImg;

	sod_box *box = 0;
	sod_box targetBox;
	int i,j,k, nbox;
	int y,x,h,w, radarValidated;
	int index;
//	float gray1, gray2;
	uint32_t dst_index;
	uint32_t src_index;
	float pixelVariance;
	float backgroundPixel_grey;
	float newPixel_grey;
	// the image coming in is in format of RGBX.
	//temp file data needs to store it as  RGBX


//	CAMERA_LED_ON;		// #1
//	CAMERA_LED_OFF;


//	CAMERA_LED_ON;		// #2
	// make a copy of the data in a space that the SOD understands.
	//  HOWEVER!!! THIS COPY IS A DIVIDE BY 2.  SO OUR IMAGE IS 1/4 THE SIZE.
	//					DIVIDE BY 2			DIVIDE BY 2			DIVIDE BY 2			//
	//								imgInCopy IS NOW 160 X 120						//

	//		AT THE SAME TIME (WOW) WE'RE GOING TO TURN IT INTO GREYSACLE.			//

	imgInCopy.c = 4;		// The input is XRGB...
	imgInCopy.h = 240;		// the input is 240, but we ++ by 2 here... we'll fix it after the loop.
	imgInCopy.w = 320;		// the input is 320, but we ++ by 2 here... we'll fix it after the loop.
	imgInCopy.data = &(imageInGreyScale[0]);
	dst_index = 0;
	for (j = 0; j < imgInCopy.h; j = j + 2) {
		for (i = 0; i < imgInCopy.w; i = i + 2) {

			k = 0;
			src_index = k + imgInCopy.c * i + imgInCopy.c * imgInCopy.w * j;
			imgInCopy.data[dst_index] = (fabs(((float)image[src_index] / 255.) - backgroundImage[dst_index][0])  ) * 0.587;	// r
			backgroundImage[dst_index][0] = backgroundImage[dst_index][0] * (BACKGROUND_IMAGE_TIME_CONSTANT) + ((float)image[src_index] / 255.) * (1 - BACKGROUND_IMAGE_TIME_CONSTANT);


			k = 1;
			src_index = k + imgInCopy.c * i + imgInCopy.c * imgInCopy.w * j;
			imgInCopy.data[dst_index] += (fabs(((float)image[src_index] / 255.) - backgroundImage[dst_index][1])  ) * 0.299;	// g
			backgroundImage[dst_index][1] = backgroundImage[dst_index][1] * (BACKGROUND_IMAGE_TIME_CONSTANT) + ((float)image[src_index] / 255.) * (1 - BACKGROUND_IMAGE_TIME_CONSTANT);

			k = 2;
			src_index = k + imgInCopy.c * i + imgInCopy.c * imgInCopy.w * j;
			imgInCopy.data[dst_index] += (fabs(((float)image[src_index] / 255.) - backgroundImage[dst_index][2])  ) * 0.114;	// b
			backgroundImage[dst_index][2] = backgroundImage[dst_index][2] * (BACKGROUND_IMAGE_TIME_CONSTANT) + ((float)image[src_index] / 255.) * (1 - BACKGROUND_IMAGE_TIME_CONSTANT);

			if(imgInCopy.data[dst_index] > BINARIZE_THRESHOLD){						// binarize threshold should range from 0 (detect everything) to 1 (detect nothing).
				binarizedImage[dst_index] = 1;
			} else{
				binarizedImage[dst_index] = 0x0;
			}
			backgroundPixel_grey = backgroundImage[dst_index][0] * 0.587 + backgroundImage[dst_index][1] * 0.299 + backgroundImage[dst_index][2] * 0.114;
			newPixel_grey = ((float)image[src_index] / 255.) * 0.587 + ((float)image[src_index] / 255.) * 0.299 + ((float)image[src_index] / 255.) * 0.114;
			pixelVariance = fabs(newPixel_grey - backgroundPixel_grey);

			if(pixelVariance > ((varianceImage[dst_index]) * VARIANCE_THRESHOLD )){	/// 	300%
//					binarizedImage[dst_index] = 1;
			}
			varianceImage[dst_index] = varianceImage[dst_index] * (VARIANCE_IMAGE_TIME_CONSTANT) + pixelVariance * (1 - VARIANCE_IMAGE_TIME_CONSTANT);
			if(varianceImage[dst_index] >1){
				varianceImage[dst_index] = 1;
			}
			if(varianceImage[dst_index] < 0){
				varianceImage[dst_index] = 0;
			}


			dst_index++;
		}
	}
	imgInCopy.c = 1;
	imgInCopy.h = 120;
	imgInCopy.w = 160;

	if(CAMERA_OUTPUT_SELECTION_FROM_BLOB_DETECT == CAMERA_OUTPUT_SELECT__AFTER_BACKGROUND_SUBTRACTION){
		SOD160_120_Greyscale_to_rgbx320_240(&imgInCopy, image);
	}
#if(0)
	if(CAMERA_OUTPUT_SELECTION_FROM_BLOB_DETECT == CAMERA_OUTPUT_SELECT__BACKGROUND_IMAGE_RED){
		imgInCopy.data = &(backgroundImage[0][0]); 		// just going to borrow imgInCopy for a second.
		SOD160_120_Greyscale_to_rgbx320_240(&imgInCopy, image);
		imgInCopy.data = &(imageInGreyScale[0]);		// and now return it.
	}
#endif
	binImg.c = 1;
	binImg.h = 120;
	binImg.w = 160;
	binImg.data = &(binarizedImage[0]);

	if(CAMERA_OUTPUT_SELECTION_FROM_BLOB_DETECT == CAMERA_OUTPUT_SELECT__AFTER_DIGITIZATION){
		SOD160_120_Greyscale_to_rgbx320_240(&binImg, image);
	}
#if(0)
	if(CAMERA_OUTPUT_SELECTION_FROM_BLOB_DETECT == CAMERA_OUTPUT_SELECT__AVERAGE_VARIANCE_IMAGE){
		binImg.data = &(varianceImage[0]);		// just going to 'borrow' binImg for a second.
		SOD160_120_Greyscale_to_rgbx320_240(&binImg, image);
		binImg.data = &(binarizedImage[0]);		// and now return it..
	}
#endif
//	CAMERA_LED_OFF;
//	CAMERA_LED_ON;		// #2
	erodeImg.c = 1;
	erodeImg.h = 120;
	erodeImg.w = 160;
	erodeImg.data = &(dilatedCopyImage[0]);
	workingImg.c = 1;
	workingImg.h = 120;
	workingImg.w = 160;
	workingImg.data = &(workingCopyImage[0]);

	// data starts in binImg.  Needs to end up in erodImg.  and workingImg can be used inbetween.

	if(IMAGE_EROSION_FACTOR == 0){
		memcpy(erodeImg.data, binImg.data, (120 * 160 * sizeof(float)));
	}else if(IMAGE_EROSION_FACTOR == 1){
		sod_erode_image_to_output(binImg, erodeImg);
	}else if(!(IMAGE_EROSION_FACTOR & 0x01)){
			// even number of erossions.
		sod_erode_image_to_output(binImg, workingImg);
		for(i = 2; i < IMAGE_EROSION_FACTOR; i += 2){
			sod_erode_image_to_output(workingImg, erodeImg);
			sod_erode_image_to_output(erodeImg, workingImg);
		}
		sod_erode_image_to_output(workingImg, erodeImg);
	}else{
			// odd number of erossions
		sod_erode_image_to_output(binImg, erodeImg);
		for(i = 2; i < IMAGE_EROSION_FACTOR; i += 2){
			sod_erode_image_to_output(erodeImg, workingImg);
			sod_erode_image_to_output(workingImg, erodeImg);
		}
	}

	if(CAMERA_OUTPUT_SELECTION_FROM_BLOB_DETECT == CAMERA_OUTPUT_SELECT__AFTER_ERODE){
		SOD160_120_Greyscale_to_rgbx320_240(&erodeImg, image);
	}


//	CAMERA_LED_OFF;

	/* Dilate the binary image, say  12 times */
//	CAMERA_LED_ON;		// #3
//	dilImg = sod_dilate_image(erodeImg, IMAGE_DILATE_FACTOR);
	dilImg.c = 1;
	dilImg.h = 120;
	dilImg.w = 160;
	dilImg.data = &(dilatedCopyImage[0]);
	// data starts in erodImg.  Needs to end up in dilImg.  and workingImg can be used inbetween.
	sod_dilate_image_to_output(erodeImg, workingImg);
	sod_dilate_image_to_output(workingImg, dilImg);
	for(i = 2; i < (IMAGE_DILATE_FACTOR); i += 2){
		sod_dilate_image_to_output(dilImg, workingImg);
		sod_dilate_image_to_output(workingImg, dilImg);
	}


	if(CAMERA_OUTPUT_SELECTION_FROM_BLOB_DETECT == CAMERA_OUTPUT_SELECT__AFTER_DILATE){
		SOD160_120_Greyscale_to_rgbx320_240(&erodeImg, image);
	}


	CAMERA_LED_ON;		// #4
	sod_image_find_blobs(dilImg, &box, &nbox, filter_cb);		/* `filter_cb`: Our filter callback which was defined above and used to discard small and big regions */
	CAMERA_LED_OFF;



	/*
	 * Draw a rectangle on each extracted & validated blob region.
	 */
//	CAMERA_LED_ON;		// #5
	for (i = 0; i < nbox; i++) {

		sod_image_draw_bbox_width(imgInCopy, box[i], 5, 255., 0, 225.); /* rose box */

		// move the box... x2 in each direction.
		// our source image is 320 x 240.
// removed for GSX - Purple boxes around whole blob.
		sod_image_draw_box_on_RGBX(&(image[0]), box[i].x *2, box[i].y *2, ((box[i].x + box[i].w) *2), ((box[i].y + box[i].h) *2), 240, 320, 255, 0, 255);

		addVideoTarget(&(box[i]));

	}
//	CAMERA_LED_OFF;

	/* Cleanup */
	sod_image_blob_boxes_release(box);		// despite what you may think, we only need to call this for the box pointer... not for each box made.


	if(getDecorationEnable(VIDEO_DECORATION_TRACKINGBOXES)){
//	// draw any past targets on the image
		for (index = 0; index < MAX_VIDEO_TARGETS ; index++) {
			i = getBoxFromVideoTarget(index, &targetBox);
			targetBox.w = 5;
			targetBox.h = 5;
			if(i >= 0){	// negative number means no target.
				sod_image_draw_box_on_RGBX(&(image[0]), targetBox.x *2, targetBox.y *2, ((targetBox.x + targetBox.w) *2), ((targetBox.y + targetBox.h) *2), 240, 320, 0, 255, 0);

			}
		}
	}


	if(getDecorationEnable(VIDEO_DECORATION_TRACKING)){
	//		draw the k-filtered data
		for (index = 0; index < ACTIVE_TARGET_LIST_SIZE; index++){
			if(getKFilteredVideoTarget(index, &x, &y, &radarValidated, &h, &w) >=0){
				targetBox.w =w;//40;
				targetBox.h =h;//40;
				targetBox.x = x - (w / 2);
				targetBox.y = y - (h * 3 / 4);
				if(targetBox.x < 0) {targetBox.x = 0;}
				if(targetBox.y < 0) {targetBox.y = 0;}
				if(radarValidated){
					sod_image_draw_box_on_RGBX(&(image[0]), targetBox.x *2, targetBox.y *2, ((targetBox.x + targetBox.w) *2), ((targetBox.y + targetBox.h) *2), 240, 320, 255, 0, 0);
//					sod_image_draw_box_on_RGBX(&(image[0]), targetBox.x *2, targetBox.y *2, ((targetBox.x + targetBox.w) *2), ((targetBox.y + targetBox.h) *2), 240, 320, 255, 0, 255);
				}else{
//		Removed for GSX - non-validated tracking.			sod_image_draw_box_on_RGBX(&(image[0]), targetBox.x *2, targetBox.y *2, ((targetBox.x + targetBox.w) *2), ((targetBox.y + targetBox.h) *2), 240, 320, 0, 0, 255);
				}
			}
		}

		float radarTargetList[NUMBER_OF_RADAR_SAMPLES_FOR_UCM][3];			// 5 samples of X,Y,magnitude.
		getRadarXY((float *)&radarTargetList);

		//		draw the k-filtered RADAR data
		for (index = 0; index < ACTIVE_TARGET_LIST_SIZE; index++){

			if((radarTargetList[index][0] < -10) || (radarTargetList[index][0] > 10) ||
				(radarTargetList[index][1] <= 0) || (radarTargetList[index][1] > 10) ||
				(radarTargetList[index][2] <= 0) || (radarTargetList[index][2] > 100))
			{
				continue;
			}
			targetBox.w = 1;
			targetBox.h = 1;
			targetBox.x = ((radarTargetList[index][0] * 160 / 20)) + (160 / 2);         // x scales from +/- 10 to 320pixels.  then shifts to mid screen;
			targetBox.y = ((radarTargetList[index][1] * 120 / 10));         			// y scales from +10 to 240pixels.
			// but now y needs to scale wrt 0,0 being the top left (the way xrgb does it), not bottom left (logical sense for graphing).
			if(targetBox.y > 120){ targetBox.y = 120;}
			targetBox.y = 120 - targetBox.y;

				// little purple boxes for radar.
			sod_image_draw_box_on_RGBX(&(image[0]), targetBox.x *2, targetBox.y *2, ((targetBox.x + targetBox.w) *2), ((targetBox.y + targetBox.h) *2), 240, 320, 255, 0, 255);

		}




	}

//	CAMERA_LED_ON;		//#6
#if(0)
	float sodPixel;
	for (j = 0; j < 240; ++j) {
		for (i = 0; i < 320; ++i) {
			for (k = 0; k < 4; ++k) {
			//	index = i + 320 * j + 320 * 240*k;
				index = k + 4 * i + 4 * 320 * j;
//				if(CAMERA_OUTPUT_FROM_BLOB_DETECT == 0){
//					sodPixel = sod_img_get_pixel(imgInCopy, i,j,k);
					sodPixel = sod_img_get_pixel(imgInCopy, i/2,j/2,0);
//					sodPixel = sod_img_get_pixel(dilImg, i/2,j/2,0);
//				}
//				else if(CAMERA_OUTPUT_FROM_BLOB_DETECT == 1){
//					sodPixel = sod_img_get_pixel(imgIn, i,j,k);
//				}
//				else if(CAMERA_OUTPUT_FROM_BLOB_DETECT == 2){
//					sodPixel = sod_img_get_pixel(grayImg, i,j,k);
//				}
//				else if(CAMERA_OUTPUT_FROM_BLOB_DETECT == 3){
//					sodPixel = sod_img_get_pixel(dilImg, i,j,k);
//				}
				if(sodPixel > 1) {
					image[index] = 255;
				}else{
					image[index]  = (sodPixel * 255);    //imgInCopy.data[index] * 255;
				}
				//	im.data[dst_index] = (float)data[src_index] / 255.;
			}
		}
	}
#endif
//	CAMERA_LED_OFF;

//	sod_free_image(erodeImg);
//	sod_free_image(dilImg);

//	sod_free_image(imgIn);
//	sod_free_image(grayImg);
//	sod_free_image(imgInCopy);
//	sod_free_image(dilImg);
//	free(imgInCopy.data);
	return 0;
}


// sod image is interpreted as greyscale (or binary).
// output is then turned in to a RGBX image.
void SOD160_120_Greyscale_to_rgbx320_240(sod_img *sod, uint8_t *rgbx){
	int i,j;
	uint32_t dest_index, source_index;
	float sodPixel;
	L1CACHE_InvalidateDCacheByRange((uint32_t)rgbx, (DEMO_CAMERA_HEIGHT * DEMO_CAMERA_WIDTH * APP_CAMERA_BUFFER_BPP));

	for (j = 0; j < 240; ++j) {
		for (i = 0; i < 320; ++i) {
			dest_index = 0 + 4 * i + 4 * 320 * j;
//				sodPixel = sod_img_get_pixel(*sod, i/2,j/2,0);
				source_index = (j/4);
				source_index = source_index *320 + i / 2;
				sodPixel = sod->data[source_index];
//				if(sodPixel >1) sodPixel = 1;
//				if(sodPixel <0) sodPixel = 0;

				rgbx[dest_index + 0]  = (uint8_t) (sodPixel * 255);
				rgbx[dest_index + 1] = rgbx[dest_index + 0];
				rgbx[dest_index + 2] = rgbx[dest_index + 0];
//				rgbx[index + 1]  = (uint8_t) (sodPixel * 255);
//				rgbx[index + 2]  = (uint8_t) (sodPixel * 255);
				rgbx[dest_index + 3]  = (uint8_t) 0x00;
		}
	}


}












//__NOINIT(RAM3) uint8_t jpegOutputFile[16384];				// RAM3 is NonCacheable SDRAM area and short of Flash, is the slowest place we could put this.
//__DATA(SRAM_DTC_cm7) uint8_t jpegOutputFile[16384];			// RAM4 is DTC memory.  Address is in the range of 0x20000000 - 0x20040000
__DATA(SRAM_DTC_cm7) uint8_t jpegOutputFile[0x5000];			// RAM4 is DTC memory.  Address is in the range of 0x20000000 - 0x20040000
//__DATA(BOARD_SDRAM) uint8_t jpegOutputFile[0xF000];
extern uint32_t	isMaster;
extern uint32_t displaySyncTimeFlag;
extern uint32_t displaySyncTimeSentFlag;


// if image == NULL, we assume that our image is already witin the tempImageBuffer and we won't make the copy //
int encodeJPG(JPEGIMAGE *jpg, uint8_t *image, uint8_t encodeQuality)
{
	// jpg encoding.
	//https://github.com/bitbank2/JPEGENC

	JPEGENCODE jpe;
	int rc,i;
	uint32_t index, colIndex;
	int iWidth,iHeight,iMCUCount, iPitch, iBytePP;

	memset(jpg,0,sizeof(JPEGIMAGE));
	memset(jpegOutputFile, 0, sizeof(jpegOutputFile));
	jpg->pOutput = jpegOutputFile;
	jpg->iBufferSize = sizeof(jpegOutputFile);
	jpg->pHighWater = &(jpg->pOutput[jpg->iBufferSize - 512]);


	if(image != NULL){
		for(index = 0; index < sizeof(jpgTempImageBuffer)/4; index ++){
			((uint32_t *)jpgTempImageBuffer)[index]  = ((uint32_t *)image)[index];

		}
	}
	if((getDecorationEnable(VIDEO_DECORATION_IP) || (getDecorationEnable(VIDEO_DECORATION_NODE))) ){
#define DECORATION_START_ROW_TL_pixels		1		// units are pixels
#define DECORATION_START_COL_TL_pixels		5		// units are pixels

		for(index = 0; index < CHARACTER_HEIGHT; index++){
			for(colIndex = 0; colIndex < (DECORATION_MASK_TL_CHARACTER_COUNT * CHARACTER_WIDTH); colIndex ++){
				if((decorationMask_TL[index][colIndex]) == 1){	// 1 means WHITE
					((uint32_t *)jpgTempImageBuffer)[((index + DECORATION_START_ROW_TL_pixels) * DEMO_CAMERA_WIDTH) + colIndex + DECORATION_START_COL_TL_pixels]  = 0x00FFFFFF;
				}else if((decorationMask_TL[index][colIndex]) == 2){	// 2 means BLACK
					((uint32_t *)jpgTempImageBuffer)[((index + DECORATION_START_ROW_TL_pixels) * DEMO_CAMERA_WIDTH) + colIndex + DECORATION_START_COL_TL_pixels]  = 0x00000000;
				}
			}
		}

	}

	if(getDecorationEnable(VIDEO_DECORATION_TIME)){

		char maskText[DECORATION_MASK_TR_CHARACTER_COUNT];
		uint8_t	hh_mm_ss[3];

		getHHMMSS(&(hh_mm_ss[0]), &(hh_mm_ss[1]), &(hh_mm_ss[2]));
		if(isMaster){
			sprintf(maskText, "*%u:%02u:%02u", hh_mm_ss[0],hh_mm_ss[1], hh_mm_ss[2]);
		}else{
			sprintf(maskText, "%u:%02u:%02u", hh_mm_ss[0],hh_mm_ss[1], hh_mm_ss[2]);
		}
		i = strlen(maskText);

		if((displaySyncTimeFlag) && (displaySyncTimeSentFlag)){
			maskText[i] = CP437_DebugBoxHighAndLow;
			maskText[i + 1] = 0;
		}else if(displaySyncTimeFlag){
			maskText[i] = CP437_DebugBoxLow;
			maskText[i + 1] = 0;
		}else if(displaySyncTimeSentFlag){
			maskText[i] = CP437_DebugBoxHigh;
			maskText[i + 1] = 0;
		}else{
			maskText[i] = ' ';
			maskText[i + 1] = 0;
		}
		displaySyncTimeFlag = 0;
		displaySyncTimeSentFlag = 0;

		generateDecorationMask(maskText, strlen(maskText), 1);


#define DECORATION_START_ROW_TR_pixels		1		// units are pixels
#define DECORATION_START_COL_TR_pixels		220		// units are pixels

		for(index = 0; index < CHARACTER_HEIGHT; index++){
			for(colIndex = 0; colIndex < (DECORATION_MASK_TR_CHARACTER_COUNT * CHARACTER_WIDTH); colIndex ++){
				if((decorationMask_TR[index][colIndex]) == 1){		// 1 = white
					((uint32_t *)jpgTempImageBuffer)[((index + DECORATION_START_ROW_TR_pixels) * DEMO_CAMERA_WIDTH) + colIndex + DECORATION_START_COL_TR_pixels]  = 0x00FFFFFF;
				}else if((decorationMask_TR[index][colIndex]) == 2){  // 2 = black
					((uint32_t *)jpgTempImageBuffer)[((index + DECORATION_START_ROW_TR_pixels) * DEMO_CAMERA_WIDTH) + colIndex + DECORATION_START_COL_TR_pixels]  = 0x00000000;
				}
			}
		}

	}




//	iWidth - image width in pixels
//	iHeight - image height in pixels
//	ucPixelType - one of JPEG_PIXEL_GRAYSCALE, JPEG_PIXEL_RGB565, JPEG_PIXEL_RGB888, or JPEG_PIXEL_ARGB8888
//	ucSubSample - one of JPEG_SUBSAMPLE_444 (none) or JPEG_SUBSAMPLE_420 (2:1 color subsampling)
//	ucQFactor - the quality level has 4 options: JPEG_Q_BEST, JPEG_Q_HIGH, JPEG_Q_MED and JPEG_Q_LOW
	// MCU is "minimum coded unit".

	// as of 2022 09 14, ucQFactor BEST = 28KBytes, HIGH = 18KBytes, MED = 11KBytes (all are approx and vary somewhat).
	rc = JPEGEncodeBegin(jpg, &jpe, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, JPEG_PIXEL_ARGB8888, JPEG_SUBSAMPLE_444, encodeQuality);

	iWidth = DEMO_CAMERA_WIDTH;
	iHeight = DEMO_CAMERA_HEIGHT;
    iMCUCount = ((iWidth + jpe.cx-1)/ jpe.cx) * ((iHeight + jpe.cy-1) / jpe.cy);

    iBytePP = 4;		// this is 'bytes per pixel'.
    iPitch = iBytePP * iWidth;		// pitch is defined as number of bytes per line.
    for (i=0; i<iMCUCount && rc == JPEG_SUCCESS; i++) {

    	rc |= JPEGAddMCU(jpg, &jpe, &(jpgTempImageBuffer[(jpe.x * iBytePP) + (jpe.y * iPitch)]), iPitch);
    	if (rc == JPEG_NO_BUFFER){
    		return (int) JPEG_NO_BUFFER;
    	}

	}
	rc = JPEGEncodeEnd(jpg);	// if successful, RC will equal file size.
	if(rc >0){
		return rc;
	}
	return -1;

}








// character mask can be up to 24 characters. 9 pixels accross x 14 pixels high for each character.
uint8_t decorationMask_TL[CHARACTER_HEIGHT][(DECORATION_MASK_TL_CHARACTER_COUNT*CHARACTER_WIDTH)];
uint8_t decorationMask_TR[CHARACTER_HEIGHT][(DECORATION_MASK_TR_CHARACTER_COUNT*CHARACTER_WIDTH)];


extern const uint8_t codePage437[18][CHARACTER_HEIGHT][CHARACTER_WIDTH];

//maskID = 0 -> decorationMask_TL
//maskID = 1 -> decorationMask_TR
void generateDecorationMask(char *text, uint32_t length, uint32_t maskID){
	uint32_t index;
	uint32_t row;
	int32_t cp437index;


	if(maskID == 0){
		if(length >= DECORATION_MASK_TL_CHARACTER_COUNT){
			length = DECORATION_MASK_TL_CHARACTER_COUNT;
		}
		memset(decorationMask_TL, 0, sizeof(decorationMask_TL));
	}else{
		if(length >= DECORATION_MASK_TR_CHARACTER_COUNT){
			length = DECORATION_MASK_TR_CHARACTER_COUNT;
		}
		memset(decorationMask_TR, 0, sizeof(decorationMask_TR));
	}


	for(index = 0; index < length; index++){

		if(isdigit(text[index])){
			cp437index = text[index] - 0x30;
		}else if(text[index] == ':'){
			cp437index = CP437_Collon;
		}else if(text[index] == '.'){
			cp437index = CP437_Period;
		}else if(text[index] == 'N'){
			cp437index = CP437_N;
		}else if(text[index] == ' '){
			cp437index = CP437_Space;
		}else if(text[index] == '*'){
			cp437index = CP437_Asterisk;
		}else if(text[index] == CP437_DebugBoxHigh){
			cp437index = CP437_DebugBoxHigh;
		}else if(text[index] == CP437_DebugBoxLow){
			cp437index = CP437_DebugBoxLow;
		}else if(text[index] == CP437_DebugBoxHighAndLow){
			cp437index = CP437_DebugBoxHighAndLow;
		}else{
			cp437index = CP437_Space; continue;
		}

		if(maskID == 0){
			for(row = 0; row< CHARACTER_HEIGHT; row++){
				memcpy(&(decorationMask_TL[row][(index * CHARACTER_WIDTH)]), codePage437[cp437index][row], CHARACTER_WIDTH);
			}
		}else {
			// note that this is right justified.
			for(row = 0; row< CHARACTER_HEIGHT; row++){
				memcpy(&(decorationMask_TR[row][((index + (DECORATION_MASK_TR_CHARACTER_COUNT - length)) * CHARACTER_WIDTH)]), codePage437[cp437index][row], CHARACTER_WIDTH);
			}
		}
	}
}

struct STR_VideoTarget{
	float x_m;					// x and y converted to meters from sensor.
	float y_m;
	uint32_t	x_pixel;		// x and y in pixel space.
	uint32_t	y_pixel;
	uint32_t targetTimeout;			// when timeout = 0 this target is gone and no longer valid.

	uint32_t	box_x_pixel;		// x and y in pixel space.
	uint32_t	box_y_pixel;
	uint32_t	box_height;			// x and y in pixel space.
	uint32_t	box_width;

}videoTargetList[MAX_VIDEO_TARGETS];



void flushVideoTargets(void){


	xSemaphoreTake( xVideoTargetMutex, ( TickType_t ) portMAX_DELAY );

	memset(videoTargetList, 0, sizeof(videoTargetList));

	xSemaphoreGive(xVideoTargetMutex);
}
#define PIXEL_TO_METER_CONVERSION_X (configParam.reserved0f )
#define PIXEL_TO_METER_CONVERSION_Y	(configParam.reserved1f )



void addVideoTarget(sod_box *box){
	uint32_t i, minTimeout, minTimeoutIndex;

	minTimeout = VIDEO_TARGET_TIMEOUT;
	minTimeoutIndex = 0;

	xSemaphoreTake( xVideoTargetMutex, ( TickType_t ) portMAX_DELAY );

	for(i = 0; i < MAX_VIDEO_TARGETS; i++){
		if(videoTargetList[i].targetTimeout == 0)
		{
			break;
		}else if(videoTargetList[i].targetTimeout < minTimeout){
			minTimeout = videoTargetList[i].targetTimeout;
			minTimeoutIndex = i;
		}
	}
	if(i == MAX_VIDEO_TARGETS){
		i = minTimeoutIndex;
	}

	videoTargetList[i].targetTimeout = VIDEO_TARGET_TIMEOUT;
	// .box gives us top left corner.
	videoTargetList[i].x_pixel = box->x + (box->w / 2);				// midspan of the rectangle in the x direction.
	videoTargetList[i].y_pixel = box->y + ((box->h * 3) / 4);			// y is best at 3/4 of the way down, I think at least.
	if(videoTargetList[i].x_pixel > 320){
		videoTargetList[i].x_pixel = 320;}
	if(videoTargetList[i].y_pixel < 0){
		videoTargetList[i].y_pixel = 0;}


	videoTargetList[i].box_x_pixel = box->x;		// x and y in pixel space.
	videoTargetList[i].box_y_pixel = box->y;
	videoTargetList[i].box_height = box->h;			// x and y in pixel space.
	videoTargetList[i].box_width = box->w;




	// convert pixels to meters.
	videoTargetList[i].x_m = videoTargetList[i].x_pixel * PIXEL_TO_METER_CONVERSION_X;
	videoTargetList[i].y_m = videoTargetList[i].y_pixel * PIXEL_TO_METER_CONVERSION_Y;



	xSemaphoreGive(xVideoTargetMutex);

}
uint32_t getBoxFromVideoTarget(uint32_t index, sod_box *targetBox){

	if(index > MAX_VIDEO_TARGETS) return -1;
	if(index < 0) return -1;
	if(videoTargetList[index].targetTimeout <= 0) return -1;


	targetBox->x = videoTargetList[index].x_pixel;
	targetBox->y = videoTargetList[index].y_pixel;
	targetBox->score = 0;
	targetBox->h = videoTargetList[index].box_height;
	targetBox->w = videoTargetList[index].box_width;

//	the rest of the 'box' variables aren't important for our use..
	return index;
}


void serviceVideoTargetList(void){
	int i,j;
	int targetIndex;
	float floatX = 0, floatY = 0;
	struct STR_VideoTarget videoTargetListCopy[MAX_VIDEO_TARGETS];
	struct STR_Jack_CameraTarget cameraTargetListForJack[MAX_VIDEO_TARGETS];

	memset(videoTargetListCopy, 0, sizeof(videoTargetListCopy));
	memset(cameraTargetListForJack, 0 , sizeof(cameraTargetListForJack));

	xSemaphoreTake( xVideoTargetMutex, ( TickType_t ) portMAX_DELAY );
	memcpy(videoTargetListCopy, videoTargetList, sizeof(videoTargetListCopy));
	for(i = 0; i < MAX_VIDEO_TARGETS; i++){
		if(videoTargetList[i].targetTimeout > 0){
			videoTargetList[i].targetTimeout--;
		}else{
			videoTargetList[i].x_m = 0;
			videoTargetList[i].y_m = 0;
			videoTargetList[i].x_pixel = 0;
			videoTargetList[i].y_pixel = 0;
			videoTargetList[i].box_height = 0;
			videoTargetList[i].box_width = 0;
		}
	}
	xSemaphoreGive(xVideoTargetMutex);
#if(1)
	j = 0;
	for(i = 0; i < MAX_VIDEO_TARGETS; i++){
		if(videoTargetListCopy[i].targetTimeout != VIDEO_TARGET_TIMEOUT){
			videoTargetListCopy[i].x_m = 0;
			videoTargetListCopy[i].y_m = 0;
			videoTargetListCopy[i].x_pixel = 0;
			videoTargetListCopy[i].y_pixel = 0;
			videoTargetListCopy[i].box_height = 0;
			videoTargetListCopy[i].box_width = 0;
			continue;
		}
		// now we have only the new targets.
		cameraTargetListForJack[j].backgroundIntensity = 0;
		cameraTargetListForJack[j].varianceIntensity = 0;
		cameraTargetListForJack[j].boxTopLeft_X = videoTargetListCopy[i].x_pixel;
		cameraTargetListForJack[j].boxTopLeft_Y = videoTargetListCopy[i].y_pixel;
		cameraTargetListForJack[j].box_h = videoTargetListCopy[i].box_height;
		cameraTargetListForJack[j].box_w = videoTargetListCopy[i].box_width;
		j++;
		targetIndex = kalmanVideo_MultiTarget_TargetListLookup(videoTargetListCopy[i].x_pixel, videoTargetListCopy[i].y_pixel);		// we're doing a lookup on the x,y pixels right now.
	//	activeTargetMarker[targetIndex]++;	// mark that this target is in existance.

		floatX = kFilter(&(videoKalmanVariables[targetIndex][0]), (float) videoTargetListCopy[i].x_pixel, videoTargetListCopy[i].box_width);			// we're borrowing this funcion call from the radar.  but it's the same stuff.
		floatY = kFilter(&(videoKalmanVariables[targetIndex][1]), (float) videoTargetListCopy[i].y_pixel, videoTargetListCopy[i].box_height);
	}
#endif
	enqueueCameraElement(&(cameraTargetListForJack[0]), MAX_VIDEO_TARGETS );

	for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i++){
		if(videoKalmanVariables[i][0].targetTimeout == 1){
			kFilter_resetTarget(&(videoKalmanVariables[i][0]));
			kFilter_resetTarget(&(videoKalmanVariables[i][1]));
		}

		if(videoKalmanVariables[i][0].targetTimeout > 0){
			videoKalmanVariables[i][0].targetTimeout--;
			videoKalmanVariables[i][1].targetTimeout--;
		}
		if(videoKalmanVariables[i][0].targetTimeout > 0){
			videoKalmanVariables[i][0].targetTimeout--;
			videoKalmanVariables[i][1].targetTimeout--;
		}
	}

}
uint32_t radarValidatedTarget[ACTIVE_TARGET_LIST_SIZE] = {0,0,0,0,0};
int getKFilteredVideoTarget(int index, int *x, int *y, int *radarValidated, int *h, int *w){

	if(index > ACTIVE_TARGET_LIST_SIZE){
		return -1;
	}
	if(videoKalmanVariables[index][0].targetTimeout == 0){
		return -1;
	}
	if(radarValidatedTarget[index]){
		*radarValidated = 1;
	}else{
		*radarValidated = 0;
	}

	*w = videoKalmanVariables[index][0].size;
	*h = videoKalmanVariables[index][1].size;
	*x = videoKalmanVariables[index][0].xhat_correction;
	*y = videoKalmanVariables[index][1].xhat_correction;
	return 0;
}



// if we look at the number below as a squared value, we don't need to do a square root calculation below.
//#define MAX_TARGET_DISTANCE_TO_BE_DECLARED_SAME_TARGET_SQUARED_m2			(MAX_TARGET_DISTANCE_TO_BE_DECLARED_SAME_TARGET_m * MAX_TARGET_DISTANCE_TO_BE_DECLARED_SAME_TARGET_m)

#define VIDEO_MULIT_TARGET_SPACING__PIXELS			(configParam.reserved5i)
#define VIDEO_KALMAN_TARGET_RESET VIDEO_TARGET_TIMEOUT

#define A_LARGE_NUMBER		1000000
uint32_t kalmanVideo_MultiTarget_TargetListLookup(float newX, float newY){
	int index;
	float targetDistanceSquared[ACTIVE_TARGET_LIST_SIZE];
	uint32_t bestMatch = 0;
	uint32_t bestMatchValue = A_LARGE_NUMBER;
	uint32_t optionToDelete = 0;
	uint32_t allTargetsInUseFlag = 1;
	float_t maxTargetDistanceToDeclareSameSqrd = (VIDEO_MULIT_TARGET_SPACING__PIXELS * VIDEO_MULIT_TARGET_SPACING__PIXELS);

	for(index = 0; index < ACTIVE_TARGET_LIST_SIZE; index++){
		if((videoTargetList[index].targetTimeout > 0) && (videoTargetList[index].targetTimeout < VIDEO_KALMAN_TARGET_RESET)){
			targetDistanceSquared[index] = ((videoKalmanVariables[index][0].xhat_correction - newX) * (videoKalmanVariables[index][0].xhat_correction - newX))
					+ ((videoKalmanVariables[index][1].xhat_correction - newY) * (videoKalmanVariables[index][1].xhat_correction - newY));

			if(targetDistanceSquared[index] > maxTargetDistanceToDeclareSameSqrd){
				targetDistanceSquared[index] += A_LARGE_NUMBER;
			}else{
				// this spot is an option to place this target.  So we'll clear the flag.
				allTargetsInUseFlag = 0;
			}

		}else{
			targetDistanceSquared[index] = A_LARGE_NUMBER;		// large number
			allTargetsInUseFlag = 0;	// clear the flag, because this spot is an option to delete.
			optionToDelete = index;
		}
	}

	bestMatch = 0;

	// this isn't as much an 'all targets in use flag' as it is a 'no target fits this bill, and there's no new hole to put it in'

	if(allTargetsInUseFlag){
		// delete the one with the longestTimeout.
		for(index = 0; index < ACTIVE_TARGET_LIST_SIZE; index++){
			if(videoKalmanVariables[index][0].targetTimeout < videoKalmanVariables[bestMatch][0].targetTimeout){		// lowest value is the 'worst'.
				bestMatch = index;
			}
		}
		// delete the entry.  reset the k-filter.
		kFilter_resetTarget(&(videoKalmanVariables[bestMatch][0]));
		kFilter_resetTarget(&(videoKalmanVariables[bestMatch][1]));

	}else{
		// within the list, we know there's a good spot to put this.
	// search the targetDistance bottom to top to find the winner (so a tie finds its way at the top)... i don't know if it matters, but I like it better that way.

		for(index = ACTIVE_TARGET_LIST_SIZE -1; index >= 0; index--){
			if(targetDistanceSquared[index] <= bestMatchValue){
				bestMatch = index;
				bestMatchValue = targetDistanceSquared[index];
			}
		}
	// there is a scenairo where the best value wasn't good enough to match to, but there was an option to delete one.
		if(bestMatchValue > maxTargetDistanceToDeclareSameSqrd){
			// our best wasn't good enough.
			// but to save us the search, we saved the spot.
			bestMatch = optionToDelete;
		}

	}
	return bestMatch;
}










const uint8_t codePage437[18][CHARACTER_HEIGHT][CHARACTER_WIDTH] =
{
		//0
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x00,0x02,0x01,0x01,0x01,0x01,0x02,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00}
},
		//1
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x00,0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x00,0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x00,0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x00,0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x00,0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x00,0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x00,0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00}
},
		//2
{ 		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x00,0x00,0x00,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x00,0x00,0x02,0x02,0x01,0x01,0x02,0x02,0x00},
		{0x00,0x02,0x02,0x01,0x01,0x02,0x02,0x00,0x00},
		{0x02,0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x02,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x00}

},
		// 3
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x00,0x00,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x00,0x00,0x02,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x00,0x00,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x00,0x00,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00}
},
	//	4
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x02,0x02,0x02,0x02,0x00,0x00},
		{0x00,0x00,0x02,0x02,0x01,0x01,0x02,0x00,0x00},
		{0x00,0x00,0x02,0x01,0x01,0x01,0x02,0x00,0x00},
		{0x00,0x02,0x02,0x01,0x01,0x01,0x02,0x00,0x00},
		{0x00,0x02,0x01,0x00,0x01,0x01,0x02,0x00,0x00},
		{0x02,0x02,0x01,0x00,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x02,0x01,0x01,0x02,0x02,0x00},
		{0x00,0x00,0x00,0x02,0x01,0x01,0x02,0x00,0x00},
		{0x00,0x00,0x00,0x02,0x01,0x01,0x02,0x00,0x00},
		{0x00,0x00,0x00,0x02,0x02,0x02,0x02,0x00,0x00}
},
	//	5
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x02,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x02,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00}
},
	//	6
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x02,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x02,0x02,0x00,0x00},
		{0x02,0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00}
},
	//7
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x00,0x00,0x00,0x02,0x01,0x01,0x02,0x02,0x00},
		{0x00,0x00,0x02,0x02,0x01,0x01,0x02,0x00,0x00},
		{0x00,0x00,0x02,0x01,0x01,0x02,0x02,0x00,0x00},
		{0x00,0x02,0x02,0x01,0x01,0x02,0x00,0x00,0x00},
		{0x00,0x02,0x01,0x01,0x02,0x02,0x00,0x00,0x00},
		{0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00}

},
//		8
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00}

},
//		9
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x01,0x02,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x02,0x00},
		{0x02,0x02,0x01,0x01,0x01,0x01,0x02,0x02,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0x00}

},
//		.
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00}
},
//		:
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00},
		{0x00,0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00}

},
//		N
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x00,0x02,0x02,0x02,0x02},
		{0x02,0x01,0x01,0x02,0x02,0x02,0x01,0x01,0x02},
		{0x02,0x01,0x01,0x01,0x02,0x02,0x01,0x01,0x02},
		{0x02,0x01,0x01,0x01,0x02,0x02,0x01,0x01,0x02},
		{0x02,0x01,0x01,0x01,0x01,0x02,0x01,0x01,0x02},
		{0x02,0x01,0x01,0x01,0x01,0x02,0x01,0x01,0x02},
		{0x02,0x01,0x01,0x02,0x01,0x01,0x01,0x01,0x02},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x01,0x02},
		{0x02,0x01,0x01,0x02,0x02,0x01,0x01,0x01,0x02},
		{0x02,0x01,0x01,0x02,0x02,0x02,0x01,0x01,0x02},
		{0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02}

},
	// space
{
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}

},
	// asterisk
{
		{0x00,0x00,0x00,0x02,0x02,0x02,0x02,0x02,0x00},
		{0x00,0x02,0x02,0x02,0x01,0x02,0x02,0x02,0x00},
		{0x00,0x02,0x01,0x02,0x01,0x02,0x01,0x02,0x00},
		{0x02,0x02,0x02,0x01,0x01,0x01,0x00,0x02,0x02},
		{0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x02},
		{0x02,0x02,0x02,0x01,0x01,0x01,0x02,0x02,0x02},
		{0x00,0x02,0x01,0x02,0x01,0x02,0x01,0x02,0x00},
		{0x00,0x02,0x02,0x02,0x01,0x02,0x02,0x02,0x00},
		{0x00,0x00,0x00,0x02,0x02,0x02,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
},
{	// debug box low
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x00}
},
{	// debug box high
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
},
{	// debug box high and low
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00},
		{0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x00}
},



};





