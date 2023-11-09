/*
 * app_power.c
 *
 *  Created on: Sept. 15, 2022
 *      Author: tsnider
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
#include "lwip/tcp.h"
#include "lwip/sockets.h"

#include "board.h"
#include "app_microwave.h"
#include "IMD200x.h"
#include "app_jack.h"
#include "app_logging.h"
#include "app_video.h"
#include "JPEGENC.h"
#include "app_power.h"
#include "app_pir.h"

#define MINIMUM_INPUT_VOLTAGE 	10
enum POWER_STATE {POWER_BOOT, POWER_STARTUP, POWER_FUSES_OFF, POWER_Vin_UNDER_VOLTS, POWER_FAULT, POWER_FUSES_ON } currentPowerState, nextPowerState;

extern float inputVoltage;
extern float eFuseCurrent[2];
extern float eFuseCurrent_dIdT[2];
extern uint32_t ADCfastSampleMode;
void power_task(void *pvParameters)
{
	float voltage, current[2], current_dIdT[2];
	TickType_t wakeTimer = 0;
#if(1)
	uint32_t	stateTimer_mS;
	float peakCurrent[2] = {0,0};
	float peakdIdT[2] = {0,0};
	// I/O is already defined

	// Assert Shutdown on both e-fuses.
	EFUSE_A_SHUTDOWN_ASSERT;
	EFUSE_B_SHUTDOWN_ASSERT;
	// measure input voltage
	voltage = inputVoltage;
	current[0] = eFuseCurrent[0];
	current[1] = eFuseCurrent[1];
	current_dIdT[0] = eFuseCurrent_dIdT[0];
	current_dIdT[1] = eFuseCurrent_dIdT[1];
	currentPowerState = POWER_STARTUP;
	stateTimer_mS = 0;
#endif

	do{
		xTaskDelayUntil( &wakeTimer, 	50 / portTICK_PERIOD_MS );
#if(1)
		if(stateTimer_mS < 100000){
			stateTimer_mS += 50;
		}

		if(currentPowerState != nextPowerState){
			currentPowerState = nextPowerState;
			stateTimer_mS = 0;
		}

		//gather inputs.
		xSemaphoreTake( xADCMutex, ( TickType_t ) portMAX_DELAY );
			voltage = inputVoltage;
			current[0] = eFuseCurrent[0];
			current[1] = eFuseCurrent[1];
			current_dIdT[0] = eFuseCurrent_dIdT[0];
			current_dIdT[1] = eFuseCurrent_dIdT[1];
		xSemaphoreGive( xADCMutex);



		switch(currentPowerState){
			case(POWER_BOOT):
				ADCfastSampleMode = 0;
				EFUSE_A_SHUTDOWN_ASSERT;
				EFUSE_B_SHUTDOWN_ASSERT;
				nextPowerState = POWER_FUSES_OFF;
			break;
			case(POWER_FUSES_OFF):
					// Assert Shutdown on both e-fuses.
				ADCfastSampleMode = 0;
				EFUSE_A_SHUTDOWN_ASSERT;
				EFUSE_B_SHUTDOWN_ASSERT;
				ADCfastSampleMode = 0;
				peakCurrent[0] = peakCurrent[1] = 0;
				if(stateTimer_mS > 2000){
					if(voltage > MINIMUM_INPUT_VOLTAGE){
						nextPowerState = POWER_FUSES_ON;
					}else{
						nextPowerState = POWER_Vin_UNDER_VOLTS;
					}
				}
			break;
			case(POWER_Vin_UNDER_VOLTS):
				ADCfastSampleMode = 0;
				EFUSE_A_SHUTDOWN_ASSERT;
				EFUSE_B_SHUTDOWN_ASSERT;
				if(voltage > MINIMUM_INPUT_VOLTAGE){
					nextPowerState = POWER_STARTUP;
					ADCfastSampleMode = 1;
				}else{
					nextPowerState = POWER_Vin_UNDER_VOLTS;
				}
			break;
			case(POWER_STARTUP):
				ADCfastSampleMode = 1;
				if(stateTimer_mS > 500){
					EFUSE_A_SHUTDOWN_UN_ASSERT;
					EFUSE_B_SHUTDOWN_UN_ASSERT;
				}

				if(peakCurrent[0] < current[0]){peakCurrent[0] = current[0];}
				if(peakCurrent[1] < current[1]){peakCurrent[1] = current[1];}
				if(peakdIdT[0] < current_dIdT[0]){peakdIdT[0] = current_dIdT[0];}
				if(peakdIdT[1] < current_dIdT[1]){peakdIdT[1] = current_dIdT[1];}

#define CURRENT_DIDT_THRESHOLD 500
#define CURRENT_THRESHOLD 1000
#define VOLTAGE_THRESHOLD 15

				if((current_dIdT[0] > CURRENT_DIDT_THRESHOLD) ||
						(current_dIdT[1] > CURRENT_DIDT_THRESHOLD) ||
						(current[0] > CURRENT_THRESHOLD) ||
						(current[1] > CURRENT_THRESHOLD) ||
						(voltage < VOLTAGE_THRESHOLD)){

					nextPowerState = POWER_FAULT;
					EFUSE_A_SHUTDOWN_ASSERT;
					EFUSE_B_SHUTDOWN_ASSERT;

				}else if(stateTimer_mS > 5000){
					nextPowerState = POWER_FUSES_ON;
				}

			break;
			case(POWER_FAULT):
				ADCfastSampleMode = 0;
				EFUSE_A_SHUTDOWN_ASSERT;
				EFUSE_B_SHUTDOWN_ASSERT;
				break;
			case(POWER_FUSES_ON):
				ADCfastSampleMode = 0;
				EFUSE_A_SHUTDOWN_UN_ASSERT;
				EFUSE_B_SHUTDOWN_UN_ASSERT;
			break;


		}

#endif
	}while(1);





}






