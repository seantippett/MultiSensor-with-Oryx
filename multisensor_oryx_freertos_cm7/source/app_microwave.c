/*
 * app_microwave.c
 *
 *  Created on: Jun. 15, 2022
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

/* lwIP */
#include "lwip/opt.h"
#include "lwip/netifapi.h"
#include "lwip/tcpip.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/timeouts.h"

#include "board.h"
#include "app_shared.h"
#include "app_microwave.h"
#include "app_config.h"
#include "IMD200x.h"
#include "app_jack.h"
#include "app_logging.h"
#include "app_alarm.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#define ACTIVE_TARGET_LIST_SIZE		5
#define MAX_TARGET_DISTANCE_TO_BE_DECLARED_SAME_TARGET_m		3
#define MICROWAVE_BOXCAR_THRESHOLD_FOR_ALARM				(0.25)		// an appropriate indoor value is much closer to 0.3 or 0.35

//struct STR_radarConfig{
//	float_t		minRange_m;				// 0 to 50 in 0.1m increments
//	float_t		maxRange_m;				// 0 to 50 in 0.1m increments
//	float_t		minVelocity_mps;		// -7.8m/s to +7.8m/s.  0.1m/s increments.
//	float_t		maxVelocity_mps;		// -7.8m/s to +7.8m/s.  0.1m/s increments.
//	float_t		minSignal_dB;			// 0 dB (min) to 100dB (max) in 0.1dB increments.
//	float_t		maxSignal_dB;			// 0 dB (min) to 100dB (max) in 0.1dB increments.
//	uint32_t	frequencyChannel;		// 1 to 8.
//	uint32_t	falseAlarmSuppression;	// 0 = disabled, 1 = enabled.
//};


struct STR_radarConfig radarConfig;


// https://simondlevy.academic.wlu.edu/kalman-tutorial/the-extended-kalman-filter-an-interactive-tutorial-for-non-experts-part-7/

struct S_kalmanFilter kalmanVariables[ACTIVE_TARGET_LIST_SIZE][2];		//  we filter the x and y separately.
#define KALMAN_X_INDEX  0
#define KALMAN_Y_INDEX  1



struct S_microwaveKalmanBoxcarVariables{
	float boxcar[MICROWAVE_KALMAN_BOXCAR_LEN];
	int boxcarHead;
	int boxcarTail;
	float boxcarPercent;
	float boxcarThreshold;
	uint32_t timeAlive;
} microwaveBoxcar[ACTIVE_TARGET_LIST_SIZE];



void kFilterTargetList(void);
void kFilter_init(struct S_kalmanFilter *kFPtr);
float kFilter(struct S_kalmanFilter *kFPtr, float input, uint32_t size);

uint8_t discrete_microwave_event_tracker(uint8_t op_code);
extern int32_t	 pir1DataForJack, pir2DataForJack;

/* The callback function. */

static TaskHandle_t xTaskToNotify_microwave = NULL;

TimerHandle_t SwTimerHandle_microwaveFilter = NULL;
SemaphoreHandle_t xMicrowaveMutex;


__BSS(BOARD_SDRAM) struct STR_IMD2002_Targets IMD2002_Targets;

__BSS(BOARD_SDRAM) float tcpOutput  [IMD2002_NUMBER_OF_DISTANCE_BINS][IMD2002_NUMBER_OF_DISTANCE_BINS];
int newTcpOutputVal = 0;

#define MICROWAVE_OUTPUT_BOXCAR_LEN 8
uint32_t microwave_output_boxcar[MICROWAVE_OUTPUT_BOXCAR_LEN];                                   //boxcar filter
uint32_t microwave_bc_head = MICROWAVE_OUTPUT_BOXCAR_LEN-1, microwave_bc_tail=0, microwave_output_max = 0;
uint32_t microwave_output = 0;
float microwave_outputX = 0;
float microwave_outputY = 10;



#define timRound(x) (((fmod(x,1)) > 0.5) ? ( ((int)x) + 1 ) : ( ((int)x)))



extern struct IMD200x_settings IMD200xSettings ;

volatile int resetRadar = 0;

#define NUMBER_OF_RADAR_SAMPLES_FOR_UCM	5
uint32_t radarMagnitude = 0;
float radarXYMagnitude[NUMBER_OF_RADAR_SAMPLES_FOR_UCM][3];			// 5 samples of X,Y,magnitude.
float radarXYMagnitude_forUCM[NUMBER_OF_RADAR_SAMPLES_FOR_UCM][3];	//

int32_t	radarErrorFlag = 0;
int32_t	radarErrorFlagPrev = 0;

int32_t	getRadarError(void){
	int32_t err;
	err = radarErrorFlag;
	if (radarErrorFlag != 0) {
		return 1;
	} // the above is temp while we aren't caring about specific errors
	return err;
}

void setRadarConfig(struct STR_radarConfig *newConfig){

	memcpy(&radarConfig, newConfig, sizeof(radarConfig));
	resetRadar = 1;
}

void getRadarConfig(struct STR_radarConfig *activeConfig){
	memcpy(activeConfig, &radarConfig, sizeof(activeConfig));
}

uint32_t getRadarMagnitude(void){
	uint32_t rv;
	rv = radarMagnitude;
	radarMagnitude = 0;
	return rv;
}

void getRadarXY(float *XY){
	xSemaphoreTake( xMicrowaveMutex, ( TickType_t ) portMAX_DELAY );
		memcpy(XY, radarXYMagnitude_forUCM, sizeof(radarXYMagnitude_forUCM));
	xSemaphoreGive(xMicrowaveMutex);


}

void microwave_polling_task(void *pvParameters){
//	int16_t samples[96];
	uint8_t status, i;
	int16_t logging[4][10];
	TickType_t wakeTimer = 0;
	uint8_t printPrescaller = 4;
	int32_t	radarFaultDebounce = 20;
	struct IMD200x_settings newSettings;
// INIT
	memset(microwave_output_boxcar,0,sizeof(microwave_output_boxcar));
	discrete_microwave_event_tracker(INIT_EVENT);

	//struct STR_radarConfig{
	//	float_t		minRange_m;				// 0 to 50 in 0.1m increments
	//	float_t		maxRange_m;				// 0 to 50 in 0.1m increments
	//	float_t		minVelocity_mps;		// -7.8m/s to +7.8m/s.  0.1m/s increments.
	//	float_t		maxVelocity_mps;		// -7.8m/s to +7.8m/s.  0.1m/s increments.
	//	float_t		minSignal_dB;			// 0 dB (min) to 100dB (max) in 0.1dB increments.
	//	float_t		maxSignal_dB;			// 0 dB (min) to 100dB (max) in 0.1dB increments.
	//	uint32_t	frequencyChannel;		// 1 to 8.
	//	uint32_t	falseAlarmSuppression;	// 0 = disabled, 1 = enabled.
	//};
	if(configParam.radarConfig.minRange_m < 0) {configParam.radarConfig.minRange_m = 0;}
	if(configParam.radarConfig.minRange_m > 50) {configParam.radarConfig.minRange_m = 50;}

	if(configParam.radarConfig.maxRange_m < 0) {configParam.radarConfig.maxRange_m = 0;}
	if(configParam.radarConfig.maxRange_m > 50) {configParam.radarConfig.maxRange_m = 50;}

	if(configParam.radarConfig.maxRange_m < configParam.radarConfig.minRange_m){configParam.radarConfig.maxRange_m = configParam.radarConfig.minRange_m;}

	if(configParam.radarConfig.minVelocity_mps < (-7.8)) {configParam.radarConfig.minVelocity_mps = (-7.8);}
	if(configParam.radarConfig.minVelocity_mps > 7.8) {configParam.radarConfig.minVelocity_mps = 7.8;}

	if(configParam.radarConfig.maxVelocity_mps < (-7.8)) {configParam.radarConfig.maxVelocity_mps = (-7.8);}
	if(configParam.radarConfig.maxVelocity_mps > 7.8) {configParam.radarConfig.maxVelocity_mps = 7.8;}

	if(configParam.radarConfig.maxVelocity_mps < configParam.radarConfig.minVelocity_mps){configParam.radarConfig.maxVelocity_mps = configParam.radarConfig.minVelocity_mps;}

	if(configParam.radarConfig.minSignal_dB < 0) {configParam.radarConfig.minSignal_dB = 0;}
	if(configParam.radarConfig.minSignal_dB > 100) {configParam.radarConfig.minSignal_dB = 100;}

	if(configParam.radarConfig.maxSignal_dB < 0) {configParam.radarConfig.maxSignal_dB = 0;}
	if(configParam.radarConfig.maxSignal_dB > 100) {configParam.radarConfig.maxSignal_dB = 100;}

	if(configParam.radarConfig.maxSignal_dB < configParam.radarConfig.minSignal_dB){configParam.radarConfig.maxSignal_dB = configParam.radarConfig.minSignal_dB;}

	if(configParam.radarConfig.frequencyChannel < 0) {configParam.radarConfig.frequencyChannel = 0;}
	if(configParam.radarConfig.frequencyChannel > 8) {configParam.radarConfig.frequencyChannel = 8;}

	if(configParam.radarConfig.falseAlarmSuppression < 0) {configParam.radarConfig.falseAlarmSuppression = 0;}
	if(configParam.radarConfig.falseAlarmSuppression > 1) {configParam.radarConfig.falseAlarmSuppression = 1;}

	if(configParam.radarConfig.radarDistanceThreshold < 0) {configParam.radarConfig.radarDistanceThreshold = 0;}
	if(configParam.radarConfig.radarDistanceThreshold > 20) {configParam.radarConfig.radarDistanceThreshold = 20;}

	radarConfig.minRange_m = configParam.radarConfig.minRange_m;
	radarConfig.maxRange_m = configParam.radarConfig.maxRange_m;
	radarConfig.minVelocity_mps = configParam.radarConfig.minVelocity_mps;
	radarConfig.maxVelocity_mps = configParam.radarConfig.maxVelocity_mps;
	radarConfig.minSignal_dB = configParam.radarConfig.minSignal_dB;
	radarConfig.maxSignal_dB = configParam.radarConfig.maxSignal_dB;
	radarConfig.frequencyChannel = configParam.radarConfig.frequencyChannel;
	radarConfig.falseAlarmSuppression = configParam.radarConfig.falseAlarmSuppression;


	PRINTF("Radar Task Startup\r\n");




	resetRadar = 0;
	radarErrorFlag = 1;	// so we always broadcast it's status on startup.
	do{
		newSettings.range.lower = radarConfig.minRange_m;
		newSettings.range.upper = radarConfig.maxRange_m;
		newSettings.velocity.lower = radarConfig.minVelocity_mps;
		newSettings.velocity.upper = radarConfig.maxVelocity_mps;
		newSettings.signal.lower = radarConfig.minSignal_dB;
		newSettings.signal.upper = radarConfig.maxSignal_dB;
		newSettings.freqChannel = radarConfig.frequencyChannel;
		newSettings.falseAlarmSuppression = radarConfig.falseAlarmSuppression;

		do{
			vTaskDelay(5000 / portTICK_PERIOD_MS );		// delay for power up and re-attempts.

			status = IMD2002_init(&(newSettings));
			radarErrorFlagPrev = radarErrorFlag;
			radarErrorFlag = status;
			if (radarErrorFlag != radarErrorFlagPrev){
				// change in error state
				// so flag to the jack task that there's a change
				diagnostic_change_flag |= 1;
				if(radarErrorFlag != 0){
					sys_stats.sensorstat.faultRadar_count++;
				}
			}
		}while (status != 0);
		PRINTF("Radar Task RESET\r\n");
		resetRadar = 0;




		memset(&IMD2002_Targets,0,sizeof(IMD2002_Targets));

		startAcquisition();

		flushUART();
		i = 0;

		while(acquisitionRunning == 0){
			vTaskDelay(10 / portTICK_PERIOD_MS );
			xSemaphoreTake( xMicrowaveMutex, ( TickType_t ) portMAX_DELAY );
				startAcquisition();
			xSemaphoreGive( xMicrowaveMutex);
		}
		vTaskDelay(100 / portTICK_PERIOD_MS );
		wakeTimer = xTaskGetTickCount();

		for(i = 0; i< ACTIVE_TARGET_LIST_SIZE; i++){
			// init kalman filter.
			kFilter_init(&kalmanVariables[i][KALMAN_X_INDEX]);
			kFilter_init(&kalmanVariables[i][KALMAN_Y_INDEX]);

			// init boxcar.
			microwaveBoxcar[i].boxcarHead = 1;
			microwaveBoxcar[i].boxcarTail = 0;
			microwaveBoxcar[i].boxcarPercent = 0;
			microwaveBoxcar[i].boxcarThreshold = MICROWAVE_BOXCAR_THRESHOLD_FOR_ALARM; 	//(0.25);		// an appropriate indoor value is much closer to 0.3 or 0.35
		}
		PRINTF("Radar Task Acquisition Running\r\n");


		// run until we get a cmd to re-reset.
		while(resetRadar == 0){
			requestNewTargetList();

			xTaskDelayUntil( &wakeTimer, 	250 / portTICK_PERIOD_MS );

			if(receiveNewTargetList() == 0) {
				xSemaphoreTake( xMicrowaveMutex, ( TickType_t ) portMAX_DELAY );

					if(IMD200xSettings.productInfo == 2002){
						memcpy(&IMD2002_Targets.rawTargetListing2002, &IMD2002TargetList, sizeof(IMD2002TargetList));
					}else{
						memcpy(&IMD2002_Targets.rawTargetListing2000, &IMD2000TargetList, sizeof(IMD2000TargetList));
					}
				xSemaphoreGive( xMicrowaveMutex);

				if(IMD200xSettings.productInfo == 2002){
					memset(logging, 0, sizeof(logging));
					for(i = 0; i < IMD2002TargetList.ui16_nrOfTargets; i++){
						logging[0][i] = (int16_t)IMD2002TargetList.target[i].f32_incidentAngle_deg;
						if(IMD2002TargetList.target[i].f32_range_m > 25) IMD2002TargetList.target[i].f32_range_m = 25;
						logging[1][i] = (int16_t)(IMD2002TargetList.target[i].f32_range_m * 10);	// 0.1m/bit
						logging[2][i] = (int16_t)IMD2002TargetList.target[i].f32_signal_dB;
					}
				}else{
					memset(logging, 0, sizeof(logging));
					for(i = 0; i <IMD2000TargetList.ui16_nrOfTargets; i++){
						logging[0][i] = 0;
						if(IMD2002TargetList.target[i].f32_range_m > 25) IMD2002TargetList.target[i].f32_range_m = 25;
						logging[1][i] = (int16_t)(IMD2000TargetList.target[i].f32_range_m * 10); // 0.1m/bit
						logging[2][i] = (int16_t)IMD2000TargetList.target[i].f32_signal_dB;
					}
				}
				enqueueRadarElement(&(logging[0][0]), &(logging[1][0]), &(logging[2][0]), 10);
				kFilterTargetList();
				radarFaultDebounce = 20;
			}else{
				if(radarFaultDebounce > 0){
					radarFaultDebounce--;
				}else{
					resetRadar = 1;
					radarErrorFlag =1;
					diagnostic_change_flag |= 1;
						sys_stats.sensorstat.faultRadar_count++;
				}


			}

			discrete_microwave_event_tracker(TIMER_TICK_EVENT);


			if(printPrescaller> 0){	printPrescaller--;	}
			else{
				printPrescaller = 0;
				xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
	#if(0)
				PRINTF("Y, Boxcar percent, KGain: %i, %i %%, %i \r\n", (int)(kalmanVariables[1].xhat_correction * 10),
															(int)(microwaveBoxcar.boxcarPercent * 100),
															(int)(kalmanVariables[1].kgain_correction * 100)
																													);
	#endif
				xSemaphoreGive( xPrintMutex );

			}

		};	// resetRadar

	}while(1);


}



#define TARGETS_TO_LOOK_AT	5

#define MICROWAVE_KALMAN_TIMEOUT   		 (configParam.radarConfig.timeConstant)
#define MICROWAVE_KALMAN_TARGET_RESET    120	// in units of number of potential targets.  i.e. 5 targets at 0.25Hz. = 20 per second.

void kFilter_init(struct S_kalmanFilter *kFPtr){
	uint32_t i;
	kFPtr->A = 1;
	kFPtr->R = 1;
	kFPtr->xhat_prediction = 10;
	kFPtr->p_prediction = 1;
	kFPtr->p_correction = 1;
	kFPtr->kgain_correction = 0;
	kFPtr->targetTimeout = 0; 		// timeout units are in sample rate. i.e. 12 at 4Hz = 3s.;
	kFPtr->p_init = 1;

	for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i++){
		if(kFPtr == &(kalmanVariables[i][KALMAN_Y_INDEX])) {
			kFPtr->xhat_init = 10;
			kFPtr->xhat_correction = 10;
		}
		else if (kFPtr == &(kalmanVariables[i][KALMAN_X_INDEX])) {
			kFPtr->xhat_init = 0;
			kFPtr->xhat_correction = 0;
		}
	}
}

void kFilter_resetTarget(struct S_kalmanFilter *kFPtr){
	uint32_t i;
		// reset.
	kFPtr->A = 1;
	kFPtr->R = 1;
		kFPtr->xhat_prediction = 10;
		kFPtr->xhat_correction = 10;
		kFPtr->p_prediction = 1;
		kFPtr->p_correction = 1;
		kFPtr->kgain_correction = 0;
		kFPtr->targetTimeout = 0;
		for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i++){
			if(kFPtr == &(kalmanVariables[i][KALMAN_Y_INDEX])) {
				kFPtr->xhat_init = 10;
				kFPtr->xhat_correction = 10;
			}
			else if (kFPtr == &(kalmanVariables[i][KALMAN_X_INDEX])) {
				kFPtr->xhat_init = 0;
				kFPtr->xhat_correction = 0;
			}
		}
		kFPtr->size = 0;

}

float kFilter(struct S_kalmanFilter *kFPtr, float input, uint32_t size){

	// input
	kFPtr->z = input;


	// predict
	//           xhat = A * xhat
	//           p = A * p * A * 1.5		// TS.  The 1.5 factor here was found experimentally in excel.

	kFPtr->xhat_prediction = kFPtr->A * kFPtr->xhat_correction;
	kFPtr->p_prediction = kFPtr->A * kFPtr->p_correction * kFPtr->A * 1.5;

	// correct
    //            g = p / (p + r)
    //            xhat = xhat + g * (z - xhat)
    //           p = (1 - g) * p

	kFPtr->kgain_correction = kFPtr->p_prediction / (kFPtr->p_prediction + kFPtr->R);
	kFPtr->xhat_correction = kFPtr->xhat_prediction + kFPtr->kgain_correction * (kFPtr->z - kFPtr->xhat_prediction);
	kFPtr->p_correction = (1 - kFPtr->kgain_correction) * kFPtr->p_prediction;
	kFPtr->targetTimeout =  MICROWAVE_KALMAN_TARGET_RESET;

	if(kFPtr->targetTimeout == 0){
		// reset.
		kFilter_resetTarget(kFPtr);
	}
	else if (kFPtr->targetTimeout < (MICROWAVE_KALMAN_TARGET_RESET - MICROWAVE_KALMAN_TIMEOUT) ) {
		// we didn't get a target within 'Kalman timeout time', but it's not time to reset yet.
		kFPtr->p_prediction = kFPtr->p_init;
		kFPtr->xhat_prediction	= kFPtr->xhat_init;
	}else{
		// we have targets actively changing things.
	}



	kFPtr->size = size;
	return kFPtr->xhat_correction;	// filtered data.



}



// if we look at the number below as a squared value, we don't need to do a square root calculation below.
//#define MAX_TARGET_DISTANCE_TO_BE_DECLARED_SAME_TARGET_SQUARED_m2			(MAX_TARGET_DISTANCE_TO_BE_DECLARED_SAME_TARGET_m * MAX_TARGET_DISTANCE_TO_BE_DECLARED_SAME_TARGET_m)

#define A_LARGE_NUMBER		1000000
uint32_t kalman_MultiTarget_TargetListLookup(float newX, float newY){
	int index;
	float targetDistanceSquared[ACTIVE_TARGET_LIST_SIZE];
	uint32_t bestMatch = 0;
	uint32_t bestMatchValue = A_LARGE_NUMBER;
	uint32_t allTargetsInUseFlag = 1;
	float_t maxTargetDistanceToDeclareSameSqrd = (configParam.radarConfig.multiTargetTrackingSpaceing * configParam.radarConfig.multiTargetTrackingSpaceing);


	for(index = 0; index < ACTIVE_TARGET_LIST_SIZE; index++){
		if((kalmanVariables[index][0].targetTimeout > 0) && (kalmanVariables[index][0].targetTimeout < MICROWAVE_KALMAN_TARGET_RESET)){
			targetDistanceSquared[index] = ((kalmanVariables[index][0].xhat_correction - newX) * (kalmanVariables[index][0].xhat_correction - newX))
					+ ((kalmanVariables[index][1].xhat_correction - newY) * (kalmanVariables[index][1].xhat_correction - newY));

			if(targetDistanceSquared[index] > maxTargetDistanceToDeclareSameSqrd){
				targetDistanceSquared[index] += A_LARGE_NUMBER;
			}

		}else{
			targetDistanceSquared[index] = A_LARGE_NUMBER;		// large number
			allTargetsInUseFlag = 0;
		}
	}

	bestMatch = 0;
	if(allTargetsInUseFlag){
		// delete the one with the longestTimeout.
		for(index = 0; index < ACTIVE_TARGET_LIST_SIZE; index++){
			if(kalmanVariables[index][0].targetTimeout < kalmanVariables[bestMatch][0].targetTimeout){		// lowest value is the 'worst'.
				bestMatch = index;
			}
		}
		// delete the entry.  reset the k-filter.
		kFilter_resetTarget(&(kalmanVariables[bestMatch][0]));
		kFilter_resetTarget(&(kalmanVariables[bestMatch][1]));

	}else{

	// now search the targetDistance bottom to top to find the winner (so a tie finds its way at the top)... i don't know if it matters, but I like it better that way.

		for(index = ACTIVE_TARGET_LIST_SIZE -1; index >= 0; index--){
			if(targetDistanceSquared[index] <= bestMatchValue){
				bestMatch = index;
				bestMatchValue = targetDistanceSquared[index];
			}
		}
	}
	return bestMatch;
}




#define MICROWAVE_MINIMUM_DISTANCE_m	(0.5)		// we tend to get a lot of false targets in the 0m range.	(in meters)
#define MICROWAVE_MAXIMUM_DISTANCE_m 	(15)		// ignore any target with a distance further than this.		(in meters)

void kFilterTargetList(void){
int i,j, targetIndex;
float floatX, floatY;
float 	tempFloat = 0;
int boxcarPrevious[ACTIVE_TARGET_LIST_SIZE];
int activeTargetMarker[ACTIVE_TARGET_LIST_SIZE];

	for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i++){
		boxcarPrevious[i] = microwaveBoxcar[i].boxcarHead;
	}

	if(IMD2002_Targets.rawTargetListing2002.ui16_nrOfTargets > TARGETS_TO_LOOK_AT){
		IMD2002_Targets.rawTargetListing2002.ui16_nrOfTargets = TARGETS_TO_LOOK_AT;	// only look at top 5 targets.
	}

	for(i = 0; i < NUMBER_OF_RADAR_SAMPLES_FOR_UCM; i++){
		radarXYMagnitude[i][0] = 0;					// UCM variables.
		radarXYMagnitude[i][1] = 0;					// UCM variables.
		radarXYMagnitude[i][2] = 0;					// UCM variables.
	}

	memset(activeTargetMarker, 0, sizeof(activeTargetMarker));

	for(i = 0; i< IMD2002_Targets.rawTargetListing2002.ui16_nrOfTargets; i++){

		if((IMD2002_Targets.rawTargetListing2002.target[i].f32_incidentAngle_deg > (70)) ||
				(IMD2002_Targets.rawTargetListing2002.target[i].f32_incidentAngle_deg < (-70))){
//				microwaveBoxcar.boxcar[microwaveBoxcar.boxcarHead] = 0;
		}
		else if((IMD2002_Targets.rawTargetListing2002.target[i].f32_range_m < MICROWAVE_MINIMUM_DISTANCE_m) ||
						(IMD2002_Targets.rawTargetListing2002.target[i].f32_range_m > MICROWAVE_MAXIMUM_DISTANCE_m)){
//				microwaveBoxcar.boxcar[microwaveBoxcar.boxcarHead] = 0;
		}
		else {
				// x projection.
			floatX = cos(DEG_TO_RAD(IMD2002_Targets.rawTargetListing2002.target[i].f32_incidentAngle_deg + 90)) * IMD2002_Targets.rawTargetListing2002.target[i].f32_range_m;
				// y projection.
			floatY = sin(DEG_TO_RAD(IMD2002_Targets.rawTargetListing2002.target[i].f32_incidentAngle_deg + 90)) * IMD2002_Targets.rawTargetListing2002.target[i].f32_range_m;

			// take a snapshot for plotting
			if(i < NUMBER_OF_RADAR_SAMPLES_FOR_UCM){
				radarXYMagnitude[i][0] = floatX;															// UCM variables.
				radarXYMagnitude[i][1] = floatY;															// UCM variables.
				radarXYMagnitude[i][2] = IMD2002_Targets.rawTargetListing2002.target[i].f32_signal_dB;		// UCM variables.
			}

			targetIndex = kalman_MultiTarget_TargetListLookup(floatX, floatY);
			activeTargetMarker[targetIndex]++;	// mark that this target is in existance.

			floatX = kFilter(&(kalmanVariables[targetIndex][0]), floatX,0);
			floatY = kFilter(&(kalmanVariables[targetIndex][1]), floatY,0);

			if((floatY > 0) && (floatY < configParam.radarConfig.radarDistanceThreshold)){		//
				microwaveBoxcar[targetIndex].boxcar[microwaveBoxcar[targetIndex].boxcarHead] = 1;
			}else{
				microwaveBoxcar[targetIndex].boxcar[microwaveBoxcar[targetIndex].boxcarHead] = 0;
			}
			microwaveBoxcar[targetIndex].boxcarHead++;
			microwaveBoxcar[targetIndex].boxcarTail++;
			if(microwaveBoxcar[targetIndex].boxcarHead >= MICROWAVE_KALMAN_BOXCAR_LEN) {microwaveBoxcar[targetIndex].boxcarHead = 0;}
			if(microwaveBoxcar[targetIndex].boxcarTail >= MICROWAVE_KALMAN_BOXCAR_LEN) {microwaveBoxcar[targetIndex].boxcarTail = 0;}
			if(microwaveBoxcar[targetIndex].boxcarHead >= configParam.radarConfig.timeConstant) {microwaveBoxcar[targetIndex].boxcarHead = 0;}
			if(microwaveBoxcar[targetIndex].boxcarTail >= configParam.radarConfig.timeConstant) {microwaveBoxcar[targetIndex].boxcarTail = 0;}
		}
	}



	for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i ++){
		if(boxcarPrevious[i] == microwaveBoxcar[i].boxcarHead){
				// this target wasn't found in the list... i.e. his Head didn't increment.
			microwaveBoxcar[i].boxcar[microwaveBoxcar[i].boxcarHead] = 0;
			microwaveBoxcar[i].boxcarHead++;
			microwaveBoxcar[i].boxcarTail++;
			if(microwaveBoxcar[i].boxcarHead >= MICROWAVE_KALMAN_BOXCAR_LEN) {microwaveBoxcar[i].boxcarHead = 0;}
			if(microwaveBoxcar[i].boxcarTail >= MICROWAVE_KALMAN_BOXCAR_LEN) {microwaveBoxcar[i].boxcarTail = 0;}
			if(microwaveBoxcar[i].boxcarHead >= configParam.radarConfig.timeConstant) {microwaveBoxcar[i].boxcarHead = 0;}
			if(microwaveBoxcar[i].boxcarTail >= configParam.radarConfig.timeConstant) {microwaveBoxcar[i].boxcarTail = 0;}

			if(kalmanVariables[i][0].targetTimeout > 0){
				kalmanVariables[i][0].targetTimeout--;
				kalmanVariables[i][1].targetTimeout--;
			}
			if(kalmanVariables[i][1].targetTimeout > 0){
				kalmanVariables[i][0].targetTimeout--;
				kalmanVariables[i][1].targetTimeout--;
			}
		}
	}

// BOX CAR Filter.
	for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i ++){
		tempFloat = 0;
		for(j = 0; j < MICROWAVE_KALMAN_BOXCAR_LEN; j++){
			tempFloat += microwaveBoxcar[i].boxcar[j];
		}
		microwave_output = tempFloat / MICROWAVE_KALMAN_BOXCAR_LEN;
		microwaveBoxcar[i].boxcarPercent = tempFloat / MICROWAVE_KALMAN_BOXCAR_LEN;
		if(microwaveBoxcar[i].boxcarPercent > radarMagnitude){
			radarMagnitude = microwaveBoxcar[i].boxcarPercent;
		}
		if((microwaveBoxcar[i].boxcarPercent) > (microwaveBoxcar[i].boxcarThreshold)){
			discrete_microwave_event_tracker(TAP_EVENT);
		}
	}

	int bestIndex[ACTIVE_TARGET_LIST_SIZE];
	int swapID;
	for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i ++){bestIndex[i]=i;}



	for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i ++){
		microwaveBoxcar[i].timeAlive += (activeTargetMarker[i] * 50);
	}

	// sort the answers
	for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i ++){
		for(j = i; j < ACTIVE_TARGET_LIST_SIZE -1; j++){

			// look at timeAlive to see which index is best.
			if (microwaveBoxcar[bestIndex[j]].timeAlive < microwaveBoxcar[bestIndex[j + 1]].timeAlive)
			{
				swapID = bestIndex[j];
				bestIndex[j] = bestIndex[j+1];
				bestIndex[j+1] = swapID;
			}
		}
	}
	float radarXYMagnitudeCopy[ACTIVE_TARGET_LIST_SIZE][3];
	memcpy(&radarXYMagnitudeCopy, radarXYMagnitude, sizeof(radarXYMagnitudeCopy));

	for(i = 0; i < NUMBER_OF_RADAR_SAMPLES_FOR_UCM; i++){
		// fill the UCM screen top 3 responses
		if(microwaveBoxcar[bestIndex[i]].timeAlive > 0){
			floatX = kalmanVariables[bestIndex[i]][0].xhat_correction;
			floatY = kalmanVariables[bestIndex[i]][1].xhat_correction;

			// K-filtered data
			floatX = (float)((int)(floatX * 100));
			floatX = floatX / 100;					// crude rounding.
			floatY = (float)((int)(floatY * 100));
			floatY = floatY / 100;					// crude rounding.

			if(floatX > 20) floatX = 20;
			if(floatX < -20) floatX = -20;
			if(floatY > 20) floatY = 20;
			if(floatY < 0) floatY = 0;
		}else{
			floatX = 0;
			floatY = 0;
		}
		radarXYMagnitude[i][0] = floatX;
		radarXYMagnitude[i][1] = floatY;
		if((floatX == floatY) && (floatX == 0)){
			radarXYMagnitude[i][2] = 0;//microwaveBoxcar[bestIndex[i]].boxcarPercent;
		}else{
			radarXYMagnitude[i][2] = radarXYMagnitudeCopy[bestIndex[i]][2];
		}
	}
#if(0)
//	radarXYMagnitude[3][0] = radarXYMagnitude[0][0] + 3;		// +3 here is for shifting right on the plot... temporarily.
//	radarXYMagnitude[3][1] = radarXYMagnitude[0][1];
//	radarXYMagnitude[3][2] = radarXYMagnitude[0][2];
//	radarXYMagnitude[4][0] = radarXYMagnitude[1][0] + 3;		// +3 here is for shifting right on the plot... temporarily.
//	radarXYMagnitude[4][1] = radarXYMagnitude[1][1];
//	radarXYMagnitude[4][2] = radarXYMagnitude[1][2];


	radarXYMagnitude[0][0] = radarXYMagnitudeCopy[0][0];
	radarXYMagnitude[0][1] = radarXYMagnitudeCopy[0][1];
	radarXYMagnitude[0][2] = radarXYMagnitudeCopy[0][2];
	radarXYMagnitude[1][0] = radarXYMagnitudeCopy[1][0];
	radarXYMagnitude[1][1] = radarXYMagnitudeCopy[1][1];
	radarXYMagnitude[1][2] = radarXYMagnitudeCopy[1][2];
	radarXYMagnitude[2][0] = radarXYMagnitudeCopy[2][0];
	radarXYMagnitude[2][1] = radarXYMagnitudeCopy[2][1];
	radarXYMagnitude[2][2] = radarXYMagnitudeCopy[2][2];
	radarXYMagnitude[3][0] = radarXYMagnitudeCopy[3][0];
	radarXYMagnitude[3][1] = radarXYMagnitudeCopy[3][1];
	radarXYMagnitude[3][2] = radarXYMagnitudeCopy[3][2];
	radarXYMagnitude[4][0] = radarXYMagnitudeCopy[4][0];
	radarXYMagnitude[4][1] = radarXYMagnitudeCopy[4][1];
	radarXYMagnitude[4][2] = radarXYMagnitudeCopy[4][2];

#endif







	xSemaphoreTake( xMicrowaveMutex, ( TickType_t ) portMAX_DELAY );
		memcpy(radarXYMagnitude_forUCM, radarXYMagnitude, sizeof(radarXYMagnitude_forUCM));
	xSemaphoreGive( xMicrowaveMutex);

	for(i = 0; i < ACTIVE_TARGET_LIST_SIZE; i ++){
		if(microwaveBoxcar[i].timeAlive > 10000){
			microwaveBoxcar[i].timeAlive = 10000;
		}
		if(microwaveBoxcar[i].timeAlive > 0){
			microwaveBoxcar[i].timeAlive -= 1;
		}
	}

}









#define MICROWAVE_TCP_SAMPLES_TO_SEND	387
#define HEADER_SIZE 13
uint8_t	microwave_sequence_num;
void tcp_microwave_task(void *pvParameters)
{

	struct netconn *conn, *newconn;
	err_t err;
//	uint32_t j;

//	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 1000 );
	LWIP_UNUSED_ARG(pvParameters);
	/* Store the handle of the calling task. */
	xTaskToNotify_microwave = xTaskGetCurrentTaskHandle();
	//xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY );

	/* Create a new connection identifier. */
	/* Bind connection to well known port number 8. */


	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, IP_ADDR_ANY, 12);
	LWIP_ERROR("microwave task: invalid conn", (conn != NULL), return;);


	/* Tell connection to go into listening mode. */
	netconn_listen(conn);
	/* Main loop. Get sensor data and send via TCP */
	while (1)
	{
		uint32_t sent_frames = 0;
		ip_addr_t client_address;
		uint16_t client_port;
		uint8_t header[HEADER_SIZE];
//		TaskHandle_t microwave_polling_handle = NULL;
		/* Grab new connection. */
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Microwave task: awaiting new connection %p\r\n", newconn);
		xSemaphoreGive( xPrintMutex );

		err = netconn_accept(conn, &newconn);
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Microwave task: accepted new connection %p\r\n", newconn);
		xSemaphoreGive( xPrintMutex );

		err |= netconn_peer(newconn, &client_address, &client_port);
		/* Process the new connection. */
		vTaskDelay(50);
		if (err == ERR_OK) {

			do {
				vTaskDelay(10 / portTICK_PERIOD_MS );


				xSemaphoreTake( xMicrowaveMutex, ( TickType_t ) portMAX_DELAY );
				if(newTcpOutputVal == 0){
					xSemaphoreGive( xMicrowaveMutex);
					continue;
				}


				/* Print out the angle data. */
				//PRINTF("x= %2d y= %2d z= %2d\r\n", g_xAngle, g_yAngle, g_zAngle);

				header[0] = 0xFF;
				header[1] = 0xFE;
				header[2] = 0xFF;
				header[3] = 0xFE;
				header[4] = 0xFF;
				header[5] = 0xFE;
				header[6] = 0xFF;
				header[7] = 0xFE;
				header[8] = microwave_sequence_num;

				newTcpOutputVal = 0;



				memcpy(&header[9], &timestamp_us, sizeof(timestamp_us));
				err = netconn_write(newconn, &header, HEADER_SIZE, NETCONN_NOCOPY);
				err |= netconn_write(newconn, (uint8_t *)&(tcpOutput), (sizeof(tcpOutput)), NETCONN_NOCOPY);

				xSemaphoreGive( xMicrowaveMutex);
				microwave_sequence_num++;
				if(microwave_sequence_num > MAX_SEQ_NUM){
					microwave_sequence_num = 0;
				}
				if(err == ERR_OK)
				{
					sent_frames++;
				}


			}
			while(err == ERR_OK);
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Microwave task: Got EOF, looping. %u sent\r\n", sent_frames);
			xSemaphoreGive( xPrintMutex );

			/* Close connection and discard connection identifier. */
			netconn_close(newconn);
			netconn_delete(newconn);
//			vTaskDelete(microwave_polling_handle);
		}
		else{
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Microwave task: netconn connection error \"%s\"\r\n", lwip_strerr(err));
			xSemaphoreGive( xPrintMutex );
			/* Close connection and discard connection identifier. */
			netconn_close(newconn);
			netconn_delete(newconn);
		}

	}
}



#define EVENT_COUNT 		(2)		// count
#define EVENT_WINDOW 	    (1.0)		//seconds
#define ALARM_WINDOW 		(10)		// seconds
#define ALARM_HOLD_TIME 	(1)		// seconds
struct detector microwave_sensor_status;
uint8_t discrete_microwave_event_tracker(uint8_t op_code)
{
  	uint8_t ret_val = 0, j;
//  	int microwaveAlrmLED_CTRL_ARRAY[2];
 //	char tempMsg[255];


  switch (op_code){
    case(INIT_EVENT):


	  for(j = 0; j<2;j++){
	      microwave_sensor_status.alarm = 0;
	      microwave_sensor_status.alarm_timer = 0;
	      microwave_sensor_status.event_count = 0;
	      microwave_sensor_status.event_timer = 0;
	      microwave_sensor_status.event_timer_reload = TICK_PER_SEC * EVENT_WINDOW;           // EVENT_WINDOW;
	      microwave_sensor_status.alarm_timer_reload = TICK_PER_SEC * ALARM_WINDOW;                //ALARM_WINDOW;
	      microwave_sensor_status.alarm_hold_timer_reload = TICK_PER_SEC * ALARM_HOLD_TIME;        //ALARM_HOLD;
	      microwave_sensor_status.event_count_threshold = EVENT_COUNT;                             //EVENT_COUNT_ALARM;

	      microwave_sensor_status.liftime_event_count =0;
	      microwave_sensor_status.liftime_alarm_count =0;
	      microwave_sensor_status.liftime_event_TO_count =0;
	      microwave_sensor_status.uptime = 0;

	  	}


    break;

    case(TAP_EVENT):    //an event has been declared by some other detection routine


      //if this is not a new event do nothing else - this is equivalent to the peak hold state in flexzone
      if(microwave_sensor_status.event_timer != 0){
        break;
      }

      //alarm bypass is triggered by a user configuration & the aux contact being in alarm (so if configured in such a way, dont alarm when the gate is opened)
 //     if(ALARM_BYPASS){ break; }

      //reaching this point indicates a new event
      microwave_sensor_status.event_count ++;	microwave_sensor_status.liftime_event_count ++;   //increment event counters
      microwave_sensor_status.event_timer = microwave_sensor_status.event_timer_reload;             //set event window timer
      microwave_sensor_status.alarm_timer = microwave_sensor_status.alarm_timer_reload;             //set alarm window timer
      ret_val =1;                                                               //new event - transmit to base
//	  task.send_status_short_no_ack =1;

      if(microwave_sensor_status.event_count == microwave_sensor_status.event_count_threshold){     //if the event counter == the event count threshold we have an alarm
             microwave_sensor_status.alarm = 1;	microwave_sensor_status.liftime_alarm_count ++;   //set alarm state and increment alarm counter
             microwave_sensor_status.alarm_timer = microwave_sensor_status.alarm_hold_timer_reload; //set alarm window timer to the alarm hold time
//			 task.send_status_short =1;
//			 LED_On(LED_RED_PORT);

//			if(microwaveDetect_Ctrl.inhibitDetection_daytime == 0){
//				 Fmt(tempMsg, "microwave Event on board %i", boardID);
//				 new_alarm = 1;
//             	 RADAR_LED_ON;
             	 radarConfidenceForJack = 100;
             	 radarConfidenceForLogging = 100;
             	 setMicrowaveProxAlarm();
//			}
//			else{
//				 Fmt(tempMsg, "microwave Event on board %i; daytime Inhibited", boardID);
//			}
//			 WRITE_EVENT_MESSAGE(tempMsg);
      }

      else if(microwave_sensor_status.event_count > microwave_sensor_status.event_count_threshold){ //if the event counter > the event count threshold we have a the continuation of a already declared alarm
             microwave_sensor_status.event_count = microwave_sensor_status.event_count_threshold;   //set the event count back to its threshold level so it doesn't increase forever
             microwave_sensor_status.alarm_timer = microwave_sensor_status.alarm_hold_timer_reload; //reset the alarm hold time
             ret_val =0;                                                        //retriggerign existing alarm - do not transmit to base
      }
#if(0)
      //MMA_8451_ACTIVITY;                                                        //clear the activity timeout timer for the accelerometer
	if(microwave_sensor_status.event_count > 0){
//		SetCtrlVal(displayPanelHandle,GetCtrlArrayItem(microwaveAlrmLED_CTRL_ARRAY[instance],boardID * 3),1);
		if(microwave_sensor_status.event_count > 1){
//			SetCtrlVal(displayPanelHandle,GetCtrlArrayItem(microwaveAlrmLED_CTRL_ARRAY[instance],boardID * 3 + 1),1);
			if(microwave_sensor_status.event_count > 2){
//				SetCtrlVal(displayPanelHandle,GetCtrlArrayItem(microwaveAlrmLED_CTRL_ARRAY[instance],boardID * 3 + 2),1);
//				if(microwaveDetect_Ctrl.inhibitDetection_daytime == 0) {
//					SetCtrlVal(panelHandle, PANEL_LED_masterEvent, 1);
//					masterEventFlag = 1;
//					lighting[boardID] = 1;
				}
			}else{
//				SetCtrlVal(displayPanelHandle,GetCtrlArrayItem(microwaveAlrmLED_CTRL_ARRAY[instance],boardID * 3 + 2),0);
			}
		}else{
//			SetCtrlVal(displayPanelHandle,GetCtrlArrayItem(microwaveAlrmLED_CTRL_ARRAY[instance],boardID * 3 + 1),0);
//			SetCtrlVal(displayPanelHandle,GetCtrlArrayItem(microwaveAlrmLED_CTRL_ARRAY[instance],boardID * 3 + 2),0);
		}
	}else{
//		SetCtrlVal(displayPanelHandle,GetCtrlArrayItem(microwaveAlrmLED_CTRL_ARRAY[instance],boardID * 3    ),0);
//		SetCtrlVal(displayPanelHandle,GetCtrlArrayItem(microwaveAlrmLED_CTRL_ARRAY[instance],boardID * 3 + 1),0);
//		SetCtrlVal(displayPanelHandle,GetCtrlArrayItem(microwaveAlrmLED_CTRL_ARRAY[instance],boardID * 3 + 2),0);
	}
#endif
    break;

    case(TIMER_TICK_EVENT):     //a timer tick has elapsed

      if(microwave_sensor_status.event_timer) {                                           //if the event window timer is set process it
        if(--microwave_sensor_status.event_timer == 0){                                   //if the event window timer is expiring
 //         if(output_max[boardID] > 255){                                                 //collect the peak hold one last time
 //           LN_STATUS.event_mag = 255;
//          }
//          else{
 //           LN_STATUS.event_mag = output_max;
 //         }
          ret_val =1;                                                           //transmit event mag to base
		  //task.send_status_short_no_ack =1;	//ship this to save airtime
 //         output_max[boardID] =0;                                                        //clear peak hold
        }
      }

      if(microwave_sensor_status.alarm_timer){                                            //if the alarm window timer is set process it
    	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  //keep the accelerometer awake if there are any events on the books
        if(--microwave_sensor_status.alarm_timer ==0){                                    //if the alarm window timer is expiring
          if(microwave_sensor_status.alarm == 0){microwave_sensor_status.liftime_event_TO_count ++;} //if no alarm was declared then increment a event timeout counter
          microwave_sensor_status.alarm = 0;                                              //clear alarm status
//          RADAR_LED_OFF;
          microwave_sensor_status.event_count = 0;                                        //clear event count
//          LN_STATUS.event_mag = 0;                                              //clear event mag
          ret_val =1;                                                           //alarm or events have cleared - transmit to base
//		  task.send_status_short =1;


        }
      }
      microwave_sensor_status.uptime ++;                                                  //increment diagnostic uptime counter
    break;

    default:
    break;

  }

  return ret_val;
 }
