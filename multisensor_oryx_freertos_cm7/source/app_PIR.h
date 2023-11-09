/*
 * app_PIR.h
 *
 *  Created on: Jun. 26, 2022
 *      Author: tsnider
 */

#ifndef APP_PIR_H_
#define APP_PIR_H_


#include "app_shared.h"
#include "peripherals.h"




	#define PIR1_DL_CLEAR	GPIO9->DR_CLEAR = 0x00004000
	#define PIR1_DL_SET		GPIO9->DR_SET =   0x00004000
	#define PIR1_DL_DIR_OUTPUT  GPIO9->GDIR = GPIO9->GDIR | 0x00004000  //	1 = output, bit 14.
	#define PIR1_DL_DIR_INPUT  	GPIO9->GDIR = GPIO9->GDIR & 0xFFFFBFFF	//	0 = input, bit 14.

	#define PIR1_DL_DATA_IN		GPIO9->DR & 0x00004000

	#define PIR1_SL_CLEAR	GPIO9->DR_CLEAR = 0x00000002
	#define PIR1_SL_SET		GPIO9->DR_SET =   0x00000002


	#define PIR2_DL_CLEAR	GPIO9->DR_CLEAR = 0x00008000
	#define PIR2_DL_SET		GPIO9->DR_SET =   0x00008000
	#define PIR2_DL_DIR_OUTPUT  GPIO9->GDIR = GPIO9->GDIR | 0x00008000  //	1 = output, bit 15.
	#define PIR2_DL_DIR_INPUT  	GPIO9->GDIR = GPIO9->GDIR & 0xFFFF7FFF	//	0 = input, bit 15.

	#define PIR2_DL_DATA_IN		GPIO9->DR & 0x00008000

	#define PIR2_SL_CLEAR	GPIO11->DR_CLEAR = 0x00000200
	#define PIR2_SL_SET		GPIO11->DR_SET =   0x00000200



/*******************************************************************************
 * Config Definitions
 ******************************************************************************/
extern SemaphoreHandle_t xADCMutex;
extern TimerHandle_t SwTimerHandle_PIRFilter;


void SwTimerCallback_PIR(TimerHandle_t xTimer);


/* The software timer period. */
#define SW_TIMER_PERIOD_PIR_MS (50 / portTICK_PERIOD_MS)


void tcp_ADC_task(void *pvParameters);

void PIR_detect(uint32_t instance,  int32_t *pirInputData, uint32_t number_of_points);
uint8_t discrete_pir_event_tracker(uint8_t op_code, int instance);

void changePIT_Period_uS(uint32_t newTimerPeriod);

void EDMA_ADC_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);	// this is an ISR function.
//******************************  Event Codes  *********************************
#define INIT_EVENT              0
#define TAP_EVENT               1
#define TIMER_TICK_EVENT        2
#define DATA_READY_EVENT        3

//*************************  Build time config parameters  *********************
#define TICK_PER_SEC                    10              //Used to compute time base of the detect routine - if changing from 10 evaluate data width of timer variables

#define POWER_BUFFER_SIZE 4

struct str_powerBuffers{
	float	currentBuffer[2][POWER_BUFFER_SIZE];
	float 	powerBuffer[2][POWER_BUFFER_SIZE];
	float	voltageBuffer[POWER_BUFFER_SIZE];
	uint32_t	bufferIndex;
	float 	dIdt[2];
	float	dVdt;
	float 	dPdt;
	float 	voltage;
	float	current[2];
	float	sourcingPower[2];
};
extern struct str_powerBuffers powerBuffers;
extern struct detector pir_sensor_status[2];

enum enumPowerState{powerState_init,
				powerState_fusesOff,
				powerState_lowVin,
				powerState_sourcingStartup,
				powerState_sourcing,
				powerState_fault};

extern uint32_t ADC_DMA_resultBuffer[ADC_DMA_BUFFER_LENGTH];
void ADC_polling_task(void *pvParameters);

uint32_t getPYD1598Data(int32_t *pir1, int32_t *pir2);
uint32_t getLeftPIRMagnitude(void);
uint32_t getRightPIRMagnitude(void);
int32_t	getLeftPIRError(void);
int32_t	getRightPIRError(void);
#endif
