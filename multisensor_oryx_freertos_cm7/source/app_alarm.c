/*
 * app_accel.c
 *
 *  Created on: Jun. 15, 2022
 *      Author: tsnider
 */


#include <string.h>
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

#include "app_accel.h"
#include "MMA8451.h"
#include "app_jack.h"
#include "app_logging.h"
#include "app_microwave.h"
#include "app_video.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TASK_TICK_PERIOS_mS	(100)

#define PREALARM_DURATION_mS	(5000 / TASK_TICK_PERIOS_mS) // time in mS.
#define ALARM_DURATION_mS		(2500 / TASK_TICK_PERIOS_mS) // time in mS.
int microwaveProxAlarmFlag;
int pirProxAlarmFlag;
int alarmFlag;

#define FAST_LED_BLINK_ON_PERIOD_mS		  (200 / TASK_TICK_PERIOS_mS) // time in mS
#define SLOW_LED_BLINK_ON_PERIOD_mS		  (1000 / TASK_TICK_PERIOS_mS) // time in mS)

enum _ENUM_alarmDecisionState{INIT, PROXALARM, ALARM, NO_ALARM };

enum _ENUM_alarmDecisionState alarmCurrentState;



#define NUMB_CORRIDORS	3
#define LEFT_CORRIDOR		0
#define CENTER_CORRIDOR		1
#define RIGHT_CORRIDOR		2

float sensorOutput[NUMB_CORRIDORS];


float getPirOutputPercent_Left(void);
float getPirOutputPercent_Right(void);
void getRadarXY(float *XY);

float radarTargetList[NUMBER_OF_RADAR_SAMPLES_FOR_UCM][3];			// 5 samples of X,Y,magnitude.

extern uint32_t radarValidatedTarget[ACTIVE_TARGET_LIST_SIZE];
void alarm_decision_task(void *pvParameters){


	TickType_t wakeTimer = 0;
	int proxAlarmTimer = PREALARM_DURATION_mS;
	int alarmTimer = ALARM_DURATION_mS;
	int alarmLEDBlinkTimer = 0;
	uint32_t i;
	int index, x,y,h,w,a;
	float targetDirection_Radar;
	float targetDirection_Camera;


	enum _ENUM_alarmDecisionState nextState;
//	enum _ENUM_alarmDecisionState previousState;

	wakeTimer = xTaskGetTickCount();

	alarmCurrentState = INIT;
	nextState = INIT;
//	previousState = INIT;


	float pirMag[NUMB_CORRIDORS];
	float radarMag[NUMB_CORRIDORS];
	float imageMag[NUMB_CORRIDORS];



	do{
		xTaskDelayUntil( &wakeTimer, 	TASK_TICK_PERIOS_mS / portTICK_PERIOD_MS );

		// gather sensor data and apply mechanical constrains / amplifications.
		pirMag[LEFT_CORRIDOR] = getPirOutputPercent_Left();
		pirMag[CENTER_CORRIDOR] = getPirOutputPercent_Left() * 0.5 + getPirOutputPercent_Right() * 0.5;
		pirMag[RIGHT_CORRIDOR] = getPirOutputPercent_Right();


		pirMag[LEFT_CORRIDOR] = getPirOutputPercent_Left();
		pirMag[CENTER_CORRIDOR] = getPirOutputPercent_Left() * 0.5 + getPirOutputPercent_Right() * 0.5;
		pirMag[RIGHT_CORRIDOR] = getPirOutputPercent_Right();

		getRadarXY(&radarTargetList);



		memset(radarValidatedTarget, 0, sizeof(radarValidatedTarget));


		// cycle through the radar target list and see if any of them align with the camera.
		for(i = 0; i < NUMBER_OF_RADAR_SAMPLES_FOR_UCM; i++){
				// ugh... dirty... just going to calculate the direction again... even though this was
				// handed to us by the radar in the first place.  TODO: properly propigate this through.
			if((radarTargetList[i][0] == 0) || (radarTargetList[i][1] == 0)){
				continue;
			}
			targetDirection_Radar = atan(((float) radarTargetList[i][0]) / ((float) radarTargetList[i][1]));
			targetDirection_Radar = targetDirection_Radar * 180. / 3.1415926535;
				// dealing with degrees to make my head not hurt... but a simple optimization would be to deal in radians.


			for (index = 0; index < ACTIVE_TARGET_LIST_SIZE; index++){
				if(getKFilteredVideoTarget(index, &x, &y, &a, &h, &w) >=0){
					// the x,y are wrt the image (160x120)
					// shift the x to middle.
					x = x - 80;
					// now calculate
					if(y == 0){	// prevent divide by 0
						continue;
					}
					targetDirection_Camera = atan(((float) x) / ((float) y));
					targetDirection_Camera = targetDirection_Camera * 180. / 3.1415926535;

					if(fabs(targetDirection_Radar - targetDirection_Camera) < 5){
						// targets are within 5 degrees.
						radarValidatedTarget[index] =1;
					}else{
//						radarValidatedTarget[index] =0;
					}

				}
			}


		}
















		if(proxAlarmTimer > 0){ proxAlarmTimer--;}
		if(alarmTimer > 0){ alarmTimer--;}
		if(alarmLEDBlinkTimer >0){
			alarmLEDBlinkTimer--;
			if(alarmCurrentState == PROXALARM){
//				if( alarmLEDBlinkTimer > SLOW_LED_BLINK_ON_PERIOD_mS){ LED_ON_ALL; }
//				else { LED_OFF_ALL; }
			}else if(alarmCurrentState == ALARM){
//				if( alarmLEDBlinkTimer > FAST_LED_BLINK_ON_PERIOD_mS){ LED_ON_ALL; }
//				else { LED_OFF_ALL; }
			}
		}
		else{
			if(alarmCurrentState == PROXALARM){
				alarmLEDBlinkTimer = SLOW_LED_BLINK_ON_PERIOD_mS * 2;
			}else if(alarmCurrentState == ALARM){
				alarmLEDBlinkTimer = FAST_LED_BLINK_ON_PERIOD_mS * 2;
			}else{
//				LED_OFF_ALL;
			}
		}

		switch(alarmCurrentState)
		{
			case INIT:
				microwaveProxAlarmFlag = FALSE;
				pirProxAlarmFlag = FALSE;
				alarmFlag = FALSE;

				proxAlarmTimer = PREALARM_DURATION_mS;
				alarmTimer = ALARM_DURATION_mS;

				nextState = NO_ALARM;

				LED_OFF_ALL;
			break;
			case PROXALARM:
				if(proxAlarmTimer == 0){
					nextState = NO_ALARM;
				}
				if(alarmFlag == TRUE){
					nextState = ALARM;
				}
				alarmTimer = ALARM_DURATION_mS;
				if((microwaveProxAlarmFlag == TRUE) || (pirProxAlarmFlag == TRUE)){
					proxAlarmTimer = PREALARM_DURATION_mS;
				}
				microwaveProxAlarmFlag = FALSE;
				pirProxAlarmFlag = FALSE;


			break;
			case ALARM:
				if(alarmTimer == 0){
					nextState = NO_ALARM;
					if(proxAlarmTimer > 0){
						nextState = PROXALARM;
					}
				}
				if((microwaveProxAlarmFlag == TRUE) || (pirProxAlarmFlag == TRUE)){
					proxAlarmTimer = PREALARM_DURATION_mS;
				}
				if((alarmFlag == TRUE) ){
					alarmTimer = ALARM_DURATION_mS;
				}
				alarmFlag = FALSE;
				microwaveProxAlarmFlag = FALSE;
				pirProxAlarmFlag = FALSE;
				proxAlarmTimer = PREALARM_DURATION_mS;
			break;
			case NO_ALARM:

				if((microwaveProxAlarmFlag == TRUE) || (pirProxAlarmFlag == TRUE)){
					nextState = PROXALARM;
				}
				proxAlarmTimer = PREALARM_DURATION_mS;
				alarmTimer = ALARM_DURATION_mS;
			break;

		}






//		previousState = alarmCurrentState;
		alarmCurrentState = nextState;

	}while(1);



}

void setMicrowaveProxAlarm(void)
{

	microwaveProxAlarmFlag = TRUE;
}
void setPIRRProxAlarm(int LorR)
{

	pirProxAlarmFlag = TRUE;
}
void setAlarmFlag(void){
	alarmFlag = TRUE;
}
uint32_t getProxAlarm(void)
{
	return TRUE;

	if((alarmCurrentState == PROXALARM) || (alarmCurrentState == ALARM)){
		return TRUE;
	}else{
		return FALSE;
	}
}
