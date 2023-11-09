/*
 * app_accel.h
 *
 *  Created on: Jun. 18, 2022
 *      Author: tsnider
 */

#ifndef APP_MICROWAVE_H_
#define APP_MICROWAVE_H_

#include "MMA8451.h"
#include "app_shared.h"
#include "IMD200x.h"
/*******************************************************************************
 * Config Definitions
 ******************************************************************************/


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEG_TO_RAD(deg)		(((double)(deg) * (double)3.141592653589796323) / (double)180)


#define IMD2002_NUMBER_OF_DISTANCE_BINS				40
#define IMD2002_DISTANCE_BIN_HISTORY				8
#define IMD2002_MAX_AMPLITUDE_IN_DISTANCE_BIN		1


//******************************  Event Codes  *********************************
#define INIT_EVENT              0
#define TAP_EVENT               1
#define TIMER_TICK_EVENT        2
#define DATA_READY_EVENT        3

//*************************  Build time config parameters  *********************
#define TICK_PER_SEC                    10              //Used to compute time base of the detect routine - if changing from 10 evaluate data width of timer variables

#define NUMBER_OF_RADAR_SAMPLES_FOR_UCM	5

#define MICROWAVE_KALMAN_BOXCAR_LEN	(4 * 5 * 4)		// this is the window time for the boxcar.  It's in units of targets.  I.e. 4hz (microwave sampling rate) x # of potential targets.  I.e. 120 = 3s
													// 4 seconds * 5 targets / second * 4 samples per second.

//enum ENUM_IMD2002_MODE {IMD2002_MODE_init, IMD2002_MODE_running} IMD2002_mode;

struct STR_IMD2002_Targets	{
	IMD2002_TargetList_t rawTargetListing2002;	// these can be combined.... later.
	IMD2000_TargetList_t rawTargetListing2000;	// these can be combined.... later.
	float distanceBins[IMD2002_NUMBER_OF_DISTANCE_BINS][IMD2002_NUMBER_OF_DISTANCE_BINS][IMD2002_DISTANCE_BIN_HISTORY];
	float distanceBinsW_Neighbours[IMD2002_NUMBER_OF_DISTANCE_BINS][IMD2002_NUMBER_OF_DISTANCE_BINS][IMD2002_DISTANCE_BIN_HISTORY];
	float distanceBinsLPF_Output  [IMD2002_NUMBER_OF_DISTANCE_BINS][IMD2002_NUMBER_OF_DISTANCE_BINS][IMD2002_DISTANCE_BIN_HISTORY];
	float distanceBinsHPF_Output  [IMD2002_NUMBER_OF_DISTANCE_BINS][IMD2002_NUMBER_OF_DISTANCE_BINS][IMD2002_DISTANCE_BIN_HISTORY];

	int distanceBinIndex;
	int distanceBinIndex_Tminus1;	// index at time - 1
	int distanceBinIndex_Tminus2;	// index at time - 2

	int alarmCount;
};


extern struct STR_IMD2002_Targets IMD2002_Targets;
extern struct detector microwave_sensor_status;

struct S_kalmanFilter{
	float A;
	float R;

	float z; 		// system input.

	float xhat_prediction;
	float p_prediction;

	float kgain_correction;
	float xhat_correction;	// system output.
	float p_correction;

	uint32_t targetTimeout;	// after a timeout, we need to reset xhat and p.
	float xhat_init;
	float p_init;

	uint32_t	size;		// this is for a GSX request and is only used on the video feed right now.
};




extern void kFilter_resetTarget(struct S_kalmanFilter *kFPtr);
extern float kFilter(struct S_kalmanFilter *kFPtr, float input, uint32_t size);
extern void SwTimerCallback_microwave(TimerHandle_t xTimer);
extern TimerHandle_t SwTimerHandle_microwaveFilter;
void microwave_polling_task(void *pvParameters);
uint32_t getkFilteredRadarTarget(int index, int *x, int *y);

/* The software timer period. */
#define SW_TIMER_PERIOD_microwave_MS (250 / portTICK_PERIOD_MS)

extern SemaphoreHandle_t xMicrowaveMutex;
extern uint32_t acquisitionRunning;


void tcp_microwave_task(void *pvParameters);
uint32_t getRadarMagnitude(void);
int32_t	getRadarError(void);
void getRadarXY(float *XY);
#endif
