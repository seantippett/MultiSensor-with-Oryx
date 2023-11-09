/*
 * app_accel.c
 *
 *  Created on: Jun. 24, 2022
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
#include "fsl_adc_etc.h"
#include "fsl_pit.h"
#include "fsl_xbara.h"
#include "fsl_wdog.h"
#include "app_shared.h"
#include "fsl_acmp.h"

#include "pin_mux.h"
#include "clock_config.h"


#include "fsl_common.h"
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
#include "peripherals.h"
#include "app_microwave.h"
#include "IMD200x.h"
#include "app_PIR.h"
#include "app_jack.h"
#include "app_logging.h"
#include "app_power.h"
#include "app_alarm.h"
#include "app_config.h"
#include "app_board.h"


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


#define VALUES_TO_STORE_20Hz 10
float calculateSlope(float *y, float deltaX, int inputBufferSize, int firstSampleIndex);

void PYD1598Setup(void);
uint32_t getPYD1598Data(int32_t *pir1, int32_t *pir2);



static TaskHandle_t xTaskToNotify_ADC = NULL;
SemaphoreHandle_t xADCMutex;

#pragma pack(1)
struct STR_RESFIFO{
	uint16_t	result;
	uint8_t		other;
	uint8_t		channel;
};
#pragma pack(1)
union U_ADC_RESULT{
		struct STR_RESFIFO ADCResult;
		uint32_t	ADCResult_u32;
};

float pir_smoothing_filter[2] = {0};
double pir_HPF_last_sample[2][3] = {0};
double pir_HPF_last_values[2][3] = {0};

double pir_LPF_last_sample[2][3] = {0};
double pir_LPF_last_values[2][3] = {0};

float pir_envelope_vals[2] = {0,0};
float pir_accumulator[2] = {0,0};
float pir_peak_envelope_vals[2] = {0,0};

uint64_t pir_output[2]={0,0};//, env_comp =0;
//int output_max = {0,0,0,0,0};

#define PIR_OUTPUT_BOXCAR_LEN 	40//4
uint32_t pir_output_boxcar[2][PIR_OUTPUT_BOXCAR_LEN] = {0};
uint8_t pir_bc_head[2] = {PIR_OUTPUT_BOXCAR_LEN-1 , PIR_OUTPUT_BOXCAR_LEN-1};
uint8_t pir_bc_tail[2] = {0, 0};


#define PIR_EVENT_THRESHOLD	15
unsigned int eventThreshold = PIR_EVENT_THRESHOLD;
//void SwTimerCallback_PIR(TimerHandle_t xTimer)
//{
//	xSemaphoreTake( xADCMutex, ( TickType_t ) portMAX_DELAY );
//
//		samplePirFlag = 1;
//
//	xSemaphoreGive( xADCMutex);
//
//}

edma_handle_t ADC_EdmaHandle_0;

float inputVoltage;
float eFuseCurrent[2];
float eFuseCurrent_dIdT[2];
uint32_t	ADCfastSampleMode = 1;
#define CURRENT_BUFFER_SIZE 4

uint32_t ADC_dataToSend = 0;
uint32_t pirResultsIndex = 0;
uint16_t pirResults[2][VALUES_TO_STORE_20Hz * 2];
uint16_t powerMonResults[3] = {0,0,0};
acmp_dac_config_t dacConfigStruct;

struct str_powerBuffers powerBuffers;

uint32_t power_flags;

//volatile bool g_AdcConversionDoneFlag;
//volatile uint32_t g_AdcConversionValue;


__NOINIT(SRAM_OC1)  union U_PIR_MSG PIR1_MsgWord;	// these are in internal memory so that we don't need to go fetch them (helps with timing).
__NOINIT(SRAM_OC1)  union U_PIR_MSG PIR2_MsgWord;	// these are in internal memory so that we don't need to go fetch them (helps with timing).
__NOINIT(SRAM_OC1)  uint32_t PIR_MsgIndex;
__NOINIT(SRAM_OC1)  uint32_t onChipRamIndex_i,onChipRamIndex_j;

#define CONVERT_CURRENT_TO_ACMP_SETTING(current) 		((int)(((33.894) * (current)) + (0.8329)))


enum enumPowerState powerState = powerState_init;
enum enumPowerState powerStatePrev = powerState_init;
enum enumPowerState powerStateNext = powerState_init;

#if(0)	// ADC IRQ is disabled.
void ADC_ETC_DONE0_Handler(void)
{

    ADC_ETC_ClearInterruptStatusFlags(ADC_ETC_BASE, kADC_ETC_Trg0TriggerSource, kADC_ETC_Done0StatusFlagMask);
//    g_AdcConversionDoneFlag = true;
    /* Get result from the trigger source chain 0. */
//    g_AdcConversionValue = ADC_ETC_GetADCConversionValue(ADC_ETC_BASE, ADC_ETC_TRIGGER_GROUP, 0U);
    ((ADC_ETC_Type *)ADC_ETC_BASE)->TRIG[0].TRIGn_CTRL |= 0xFF000000;
    __DSB();

}
#endif







volatile uint32_t g_TransferCounter = 0;
/* User callback function for EDMA transfer. */
/* This is fired when DMA transfer is done   */
/* THIS IS DONE IN THE ISR!!!  */
void EDMA_ADC_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
	uint32_t i;
	union U_ADC_RESULT *adcResultsPtr = (union U_ADC_RESULT *)ADC_DMA_resultBuffer;
    g_TransferCounter++;

    if(currentPITPeriod == SAMPLING_PERIOD_STARTUP_uS){
		for(i = 0; i<ADC_DMA_BUFFER_LENGTH; i++){
			if(adcResultsPtr->ADCResult.channel == (0x80 | ADC_COMMAND_NUMB__IMON_A)){
				if(adcResultsPtr->ADCResult.result > 10000){
//					EFUSE_A_SHUTDOWN_ASSERT;
//					EFUSE_B_SHUTDOWN_ASSERT;
				}
			}
			if(adcResultsPtr->ADCResult.channel == (0x80 | ADC_COMMAND_NUMB__IMON_B)){
				if(adcResultsPtr->ADCResult.result > 10000){
//					EFUSE_A_SHUTDOWN_ASSERT;
//					EFUSE_B_SHUTDOWN_ASSERT;
				}
			}
			adcResultsPtr++;
		}
    }
}




// Analog comparitor.

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define IMONA_ACMP_BASEADDR         CMP1
#define IMONA_ACMP_MINUS_INPUT      1U
#define IMONA_ACMP_PLUS_INPUT       7U /*  Internal 8bit DAC output. */
#define IMONA_ACMP_IRQ_ID           ACMP1_IRQn
#define IMONA_ACMP_IRQ_HANDLER_FUNC ACMP1_IRQHandler


#define IMONB_ACMP_BASEADDR         CMP2
#define IMONB_ACMP_MINUS_INPUT      1U
#define IMONB_ACMP_PLUS_INPUT       7U /*  Internal 8bit DAC output. */
#define IMONB_ACMP_IRQ_ID           ACMP2_IRQn
#define IMONB_ACMP_IRQ_HANDLER_FUNC ACMP2_IRQHandler

#define STARTUP_DOWNSTREAM_CURRENT_LIMIT_AMPS		(0.5)

float downstreamCurrentLimit = STARTUP_DOWNSTREAM_CURRENT_LIMIT_AMPS;

#define CURRENT_TO_DAC_CONVERSION

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t IMONA_acmpOutput = 0U;
volatile uint32_t IMONB_acmpOutput = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/

void IMONA_ACMP_IRQ_HANDLER_FUNC(void)
{
    uint32_t statusFlags;

    statusFlags = ACMP_GetStatusFlags(IMONA_ACMP_BASEADDR);
    ACMP_ClearStatusFlags(IMONA_ACMP_BASEADDR, statusFlags);

    if ((kACMP_OutputAssertEventFlag == (statusFlags & kACMP_OutputAssertEventFlag)))
    {
    	IMONA_acmpOutput = 1U;
    	POWER_FAULT_A_LED_ON;
    }
    else if ((kACMP_OutputAssertEventFlag != (statusFlags & kACMP_OutputAssertEventFlag)))
    {
    	IMONA_acmpOutput = 0U;
    	POWER_FAULT_A_LED_OFF;
    }
    else
    {
        /* Unknown interrupt. */
    }
    SDK_ISR_EXIT_BARRIER;
}

void IMONB_ACMP_IRQ_HANDLER_FUNC(void)
{
    uint32_t statusFlags;

    statusFlags = ACMP_GetStatusFlags(IMONB_ACMP_BASEADDR);
    ACMP_ClearStatusFlags(IMONB_ACMP_BASEADDR, statusFlags);

    if ((kACMP_OutputAssertEventFlag == (statusFlags & kACMP_OutputAssertEventFlag)))
    {
    	IMONB_acmpOutput = 1U;
    	POWER_FAULT_B_LED_ON;
    }
    else if ((kACMP_OutputAssertEventFlag != (statusFlags & kACMP_OutputAssertEventFlag)))
    {
    	IMONB_acmpOutput = 0U;
    	POWER_FAULT_B_LED_OFF;
    }
    else
    {
        /* Unknown interrupt. */
    }
    SDK_ISR_EXIT_BARRIER;
}

void setupIMON_ACMP(void){
    acmp_config_t acmpConfigStruct;
    acmp_channel_config_t channelConfigStruct;
    acmp_discrete_mode_config_t acmpDiscreteconfig;


	   /* Configure ACMP. */
	    /*
	     * acmpConfigStruct.enableHighSpeed = false;
	     * acmpConfigStruct.enableInvertOutput = false;
	     * acmpConfigStruct.useUnfilteredOutput = false;
	     * acmpConfigStruct.enablePinOut = false;
	     * acmpConfigStruct.offsetMode = kACMP_OffsetLevel0;
	     * acmpConfigStruct.hysteresisMode = kACMP_HysteresisLevel0;
	     */
	    ACMP_GetDefaultConfig(&acmpConfigStruct);
	    ACMP_Init(IMONA_ACMP_BASEADDR, &acmpConfigStruct);
	    ACMP_Init(IMONB_ACMP_BASEADDR, &acmpConfigStruct);

	    /* Configure negative inputs are coming from 3v domain. */
	    ACMP_GetDefaultDiscreteModeConfig(&acmpDiscreteconfig);
	    acmpDiscreteconfig.enableNegativeChannelDiscreteMode = true;
	    acmpDiscreteconfig.enableResistorDivider = true;		// because our inputs come from 3V3 source.
	    ACMP_SetDiscreteModeConfig(IMONA_ACMP_BASEADDR, &acmpDiscreteconfig);
	    ACMP_SetDiscreteModeConfig(IMONB_ACMP_BASEADDR, &acmpDiscreteconfig);

	    /* Configure channel. Select the positive port input from DAC and negative port input from minus mux input. */
	    channelConfigStruct.minusMuxInput = IMONA_ACMP_MINUS_INPUT;
	    channelConfigStruct.plusMuxInput  = IMONA_ACMP_PLUS_INPUT;
	    ACMP_SetChannelConfig(IMONA_ACMP_BASEADDR, &channelConfigStruct);

	    channelConfigStruct.minusMuxInput = IMONB_ACMP_MINUS_INPUT;
	    channelConfigStruct.plusMuxInput  = IMONB_ACMP_PLUS_INPUT;
	    ACMP_SetChannelConfig(IMONB_ACMP_BASEADDR, &channelConfigStruct);

	    /* Configure DAC. */

	    dacConfigStruct.referenceVoltageSource = kACMP_VrefSourceVin2;

	    dacConfigStruct.DACValue = CONVERT_CURRENT_TO_ACMP_SETTING(downstreamCurrentLimit);  //0x7FU; /* Half of referene voltage. */

	    dacConfigStruct.workMode = kACMP_DACWorkLowSpeedMode;

	    ACMP_SetDACConfig(IMONA_ACMP_BASEADDR, &dacConfigStruct);
	    ACMP_SetDACConfig(IMONB_ACMP_BASEADDR, &dacConfigStruct);

	    /* Enable the interrupts. */
	    ACMP_EnableInterrupts(IMONA_ACMP_BASEADDR, kACMP_OutputRisingInterruptEnable | kACMP_OutputFallingInterruptEnable);
	    ACMP_EnableInterrupts(IMONB_ACMP_BASEADDR, kACMP_OutputRisingInterruptEnable | kACMP_OutputFallingInterruptEnable);
	    EnableIRQ(IMONA_ACMP_IRQ_ID);
	    EnableIRQ(IMONB_ACMP_IRQ_ID);

	    ACMP_Enable(IMONA_ACMP_BASEADDR, true);
	    ACMP_Enable(IMONB_ACMP_BASEADDR, true);

	    /* Get ACMP's output initial value. */
	    if (kACMP_OutputAssertEventFlag == (kACMP_OutputAssertEventFlag & ACMP_GetStatusFlags(IMONA_ACMP_BASEADDR))){
	        IMONA_acmpOutput = 1U;POWER_FAULT_A_LED_ON;
	    } else {
	    	IMONA_acmpOutput = 0U;POWER_FAULT_A_LED_OFF;
	    }

	    if (kACMP_OutputAssertEventFlag == (kACMP_OutputAssertEventFlag & ACMP_GetStatusFlags(IMONB_ACMP_BASEADDR))){
	        IMONB_acmpOutput = 1U;POWER_FAULT_B_LED_ON;
	    } else {
	    	IMONB_acmpOutput = 0U;POWER_FAULT_B_LED_OFF;
	    }


}












volatile int pirReset;
struct STR_PIRConfig defaultPIRConfig;
struct STR_PIRConfig PIRSettings[2];



void setPIRConfig(struct STR_PIRConfig *newConfig){

	memcpy(&PIRSettings, newConfig, sizeof(struct STR_PIRConfig));
	pirReset = 1;
}
void getPIRConfig(struct STR_PIRConfig *config){
	memcpy(config, &PIRSettings, sizeof(struct STR_PIRConfig));
}

uint32_t leftPIRMagnitude;
uint32_t rightPIRMagnitude;
int32_t	 pir1DataForJack, pir2DataForJack;
float	pirOutputPercent_Left;
float	pirOutputPercent_Right;

float getPirOutputPercent_Left(void){
	float rv;
		if(pir2DataForJack >= 0){
			rv = 1;
		}
		else{
			rv = 0;
		}
		return rv;
}
float getPirOutputPercent_Right(void){
	float rv;
		if(pir1DataForJack >= 0){
			rv = 1;
		}
		else{
			rv = 0;
		}
		return rv;
}


uint32_t getLeftPIRMagnitude(void){
	uint32_t rv;
//	rv = leftPIRMagnitude;
//	leftPIRMagnitude = 0;
	if(pir2DataForJack >= 0){
		rv = pir2DataForJack * 75;
	}
	else{
		rv = (0 - pir2DataForJack)  * 75;
	}

	return rv;
}
uint32_t getRightPIRMagnitude(void){
	uint32_t rv;
//	rv = rightPIRMagnitude;
//	rightPIRMagnitude = 0;

	if(pir1DataForJack >= 0){
		rv = pir1DataForJack  * 75;
	}
	else{
		rv = (0 - pir1DataForJack) * 75;
	}
	return rv;
}

int32_t	pirErrorFlag_Left = 0;
int32_t	pirErrorFlagPrev_Left = 0;
int32_t	pirErrorFlag_Right = 0;
int32_t	pirErrorFlagPrev_Right = 0;

int32_t	getLeftPIRError(void){
	int32_t err;
	err = pirErrorFlag_Left;
	if (pirErrorFlag_Left != 0) {
			return 1;
		} // the above is temp while we aren't caring about specific errors

	return err;
}
int32_t	getRightPIRError(void){
	int32_t err;
	err = pirErrorFlag_Right;
	if (pirErrorFlag_Right != 0) {
			return 1;
		} // the above is temp while we aren't caring about specific errors

	return err;
}

//////////////////////////////// THIS IS HOW THE ADC / DMA transfer works.///////////////////////////////////////////
// PIT timer overflows.  This cycles through the XBAR switch.  This fires the ADC_ETC (external trigger controller.
// 	ADC_ETC causes a 'chain' of ADC conversions.  The results of this go in to the hardware FIFO of the ADC.
//  HW FIFO on the ADC is 16 samples long (32 bits each - Each result contains info on which ADC CH fired it, hence 32 bits, not 12)
//	We have the ADC High Water Mark set to 10 samples.  When this happens it Sets a High Water Flag, as well as a DMA flag.
//	ADC has a direct bit connection to the DMA_MUX.  No XBAR switch required here.  DMA is setup to transfer 10 samples (unit is
//  bytes, so it's setup for 10 x 4 bytes).  High water mark on the ADC is self clearing.  DMA interrupt happens after this
//	transfer is done.  T.S. 2022 09 28
//////////////////////////////// THIS IS HOW THE ADC / DMA transfer works.///////////////////////////////////////////

void XBARA_Configuration(void);
void LPADC_Configuration(void);
void ADC_ETC_Configuration(void);
void DMA_configuration(void);
void ADC_polling_task(void *pvParameters){

	union U_ADC_RESULT ADCresult;
//	uint32_t	dIdt_debounce = 0;
//	uint32_t	dPdt_debounce = 0;
//	uint32_t	current_debounce = 0;
//	uint16_t 	temp16;
	TickType_t wakeTimer = 0;
	uint32_t 	i;
	uint32_t	ADC_DMABufferCopy[ADC_DMA_BUFFER_LENGTH];
	uint8_t		tempPIRResult_8Bit[2];
	uint32_t pirPrescaller_mS = 0;
	int32_t pir1data, pir2data;
	int32_t pirError = ERR_OK;
	struct STR_PIRConfig *pirConfigPtr;
	float powerStateTimer_mS = 0;
	power_flags = 0;


    memset(&powerBuffers, 0, sizeof(powerBuffers));
	discrete_pir_event_tracker(INIT_EVENT, 0);
	discrete_pir_event_tracker(INIT_EVENT, 1);
	PIR_LED_OFF;


	EFUSE_A_SHUTDOWN_ASSERT;
	EFUSE_B_SHUTDOWN_ASSERT;
	POWER_FAULT_A_LED_OFF;
	POWER_FAULT_B_LED_OFF;
	downstreamCurrentLimit = STARTUP_DOWNSTREAM_CURRENT_LIMIT_AMPS;
	setupIMON_ACMP();		// setup the analog comparator for current monitoring.


	EFUSE_A_SHUTDOWN_UN_ASSERT;
	EFUSE_B_SHUTDOWN_UN_ASSERT;

	changePIT_Period_uS(SAMPLING_PERIOD_STARTUP_uS);
	wakeTimer = xTaskGetTickCount();
	ADC_dataToSend = 0;

	for(i = 0 ; i < 2; i++){
		if(i == 0) {pirConfigPtr = &configParam.PIR0_Config;} else { pirConfigPtr = &configParam.PIR1_Config;}

		if(pirConfigPtr->threshold > 255){pirConfigPtr->threshold = 255;}
		if(pirConfigPtr->threshold < 0){pirConfigPtr->threshold = 0;}

		if(pirConfigPtr->blindTime_mS > 16){pirConfigPtr->blindTime_mS = 16;}
		if(pirConfigPtr->blindTime_mS < 0){pirConfigPtr->blindTime_mS = 0;}

		if(pirConfigPtr->pulseCount > 4){pirConfigPtr->pulseCount = 4;}
		if(pirConfigPtr->pulseCount < 1){pirConfigPtr->pulseCount = 1;}

		if(pirConfigPtr->windowTime_S > 4){pirConfigPtr->windowTime_S = 4;}
		if(pirConfigPtr->windowTime_S < 0){pirConfigPtr->windowTime_S = 0;}

		if(pirConfigPtr->HPFCutOff > 1){pirConfigPtr->HPFCutOff = 1;}
		if(pirConfigPtr->HPFCutOff < 0){pirConfigPtr->HPFCutOff = 0;}

		if(pirConfigPtr->countMode > 1){pirConfigPtr->countMode = 1;}
		if(pirConfigPtr->countMode < 0){pirConfigPtr->countMode = 0;}


		memcpy(&(PIRSettings[i]), pirConfigPtr, sizeof(PIRSettings[0]));
	}

	/* Initialize LPADC converter */
	LPADC_Configuration();


	/* cross bar switch */
	XBARA_Configuration();

	/* Initialize the External Trigger Controller (ETC) */
	/* This is what makes our ADC get triggered multiple times (i.e. our chain). */
	/* Note that the initial trigger still BEGINS FROM THE XBAR.  The PIT sends the*/
	/* trigger through the XBAR.  The XBAR triggers this, and then this triggers*/
	/* the ADC (multiple times, in our case)									*/
	ADC_ETC_Configuration();

	DMA_configuration();

	/* Periodic interrupt timer.  This does the timing for the ADC Samples */
	// this needs to be on, it's off now for debugging.	PIT_Configuration();
//	PIT_ClearStatusFlags(PIT_BASEADDR, PIT_CHANNEL, 1);	// not necessary.  But I left it here for future debug use.


	/* Enable the NVIC. */
	EnableIRQ(ADC_ETC_IRQ0_IRQn);


	while(1){


		PYD1598Setup();

		pirReset = 0;
		PIT_StartTimer(PIT_BASEADDR, PIT_CHANNEL);

		while (pirReset == 0)
		{
			g_TransferCounter = 0;				// this transfer counter is set in the DMA transfer from the ADC.
//			EDMA_StopTransfer(&ADC_EdmaHandle_0);	// pause transfers (this does NOT pause the ADC).
//				memcpy(ADC_DMABufferCopy, ADC_DMA_resultBuffer, sizeof(ADC_DMABufferCopy));
//				memset(ADC_DMA_resultBuffer, 0 , sizeof(ADC_DMA_resultBuffer));
//			EDMA_StartTransfer(&ADC_EdmaHandle_0);	// allow Transfers again.  Note that this method will never 'skip' a transfer.
			while (!g_TransferCounter)			// wait for DMA transfer to be complete.
			{
				if(LPADC1->STAT & 0x02){
					LPADC1->STAT |= 0x02;	// FIFO Overflow.
				}
				if(EDMA_ADC_BASEADDR->CR & 0x020){
					// halted due to DMA error.
					// clear error and stop halting.
					EDMA_ADC_BASEADDR->CR = EDMA_ADC_BASEADDR->CR & ~(0x00000030);		// I don't know why we get these.  TODO: fix this.
				}
				if(currentPITPeriod == SAMPLING_PERIOD_STARTUP_uS){
					xTaskDelayUntil( &wakeTimer, 	2 / portTICK_PERIOD_MS );
					powerStateTimer_mS += 2;			// note this MUST be less than DMA period.
					pirPrescaller_mS +=2;
				}else{
					xTaskDelayUntil( &wakeTimer, 	40 / portTICK_PERIOD_MS );
					powerStateTimer_mS += 40;			// note this MUST be less than DMA period.
					pirPrescaller_mS +=40;
				}
				if(powerStateTimer_mS > 1000000) powerStateTimer_mS = 1000000;
				if(pirPrescaller_mS >=40 ){
					break;
				}
			}
			EDMA_StopTransfer(&ADC_EdmaHandle_0);	// pause transfers (this does NOT pause the ADC).
				memcpy(ADC_DMABufferCopy, ADC_DMA_resultBuffer, sizeof(ADC_DMABufferCopy));
				memset(ADC_DMA_resultBuffer, 0 , sizeof(ADC_DMA_resultBuffer));
			EDMA_StartTransfer(&ADC_EdmaHandle_0);	// allow Transfers again.  Note that this method will never 'skip' a transfer.


			if(pirPrescaller_mS >= 40){
				pirPrescaller_mS -= 40;
				pirError = ERR_OK;
				// begin sampling.
				pirError = getPYD1598Data(&pir1data, &pir2data);

				pirErrorFlagPrev_Left = pirErrorFlag_Left;
				pirErrorFlagPrev_Right = pirErrorFlag_Right;

				if(pirError == 0){
					pirErrorFlag_Left = 0;
					pirErrorFlag_Right = 0;
				}else if(pirError == -1){
					pirErrorFlag_Left = -1;
					pirErrorFlag_Right = 0;
				}else if(pirError == -2){
					pirErrorFlag_Left = 0;
					pirErrorFlag_Right = -1;
				}else if(pirError == -3){
					pirErrorFlag_Left = -1;
					pirErrorFlag_Right = -1;
				}

				if (pirErrorFlagPrev_Left != pirErrorFlag_Left){
					// change in error state
					// so flag to the jack task that there's a change
					diagnostic_change_flag |= 1;
					if(pirErrorFlag_Left != 0){
						sys_stats.sensorstat.faultLPIR_count++;

					}
				}
				if (pirErrorFlagPrev_Right != pirErrorFlag_Right){
					// change in error state
					// so flag to the jack task that there's a change
					diagnostic_change_flag |= 1;
					if(pirErrorFlag_Right != 0){

						sys_stats.sensorstat.faultRPIR_count++;
					}
				}



				if(pirError > 0){
					// pir was reset.  Handle this with a warning, perhaps.  but the error itself is cleared now and we can carry on.
					pirError = ERR_OK;
				}
				if(pirError <0){
					// this is an error in reading.  Run Setup again.
					PYD1598Setup();
					pirError = ERR_OK;
					pir1data = pir2data = 0;
				}

				pirResults[0][pirResultsIndex] = pir1data;
				pirResults[1][pirResultsIndex] = pir2data;

//				pir1DataForJack = pir1data;
//				pir2DataForJack = pir2data;

				if(pir1data > INT8_MAX) {tempPIRResult_8Bit[0] = INT8_MAX;}
				else if(pir1data < INT8_MIN) {tempPIRResult_8Bit[0] = INT8_MIN;}
				else{tempPIRResult_8Bit[0] = pir1data;}

				if(pir2data > INT8_MAX) {tempPIRResult_8Bit[1] = INT8_MAX;}
				else if(pir2data < INT8_MIN) {tempPIRResult_8Bit[1] = INT8_MIN;}
				else{tempPIRResult_8Bit[1] = pir2data;}


				PIR_detect(0, &pir1data, 1);

				PIR_detect(1, &pir2data, 1);

				enqueuePIRElement(&(tempPIRResult_8Bit[0]), &(tempPIRResult_8Bit[1]), 1);

				pirResultsIndex++;
				if((pirResultsIndex % VALUES_TO_STORE_20Hz) == 0){
					ADC_dataToSend = (pirResultsIndex / VALUES_TO_STORE_20Hz) + 1;
				}
				if(pirResultsIndex >= VALUES_TO_STORE_20Hz * 2){
					pirResultsIndex = 0;
				}

				discrete_pir_event_tracker(TIMER_TICK_EVENT, 0);
				discrete_pir_event_tracker(TIMER_TICK_EVENT, 1);

			}




			// work through the bufferCopy and place results in to their appropriate bin.
			for(i = 0; i < (sizeof(ADC_DMABufferCopy) / 4); i++){

				ADCresult.ADCResult_u32 = ADC_DMABufferCopy[i];
				ADCresult.ADCResult.result = ADCresult.ADCResult.result >> 3;	// 12bit ADC.  It's bit shifted too.
				if(ADCresult.ADCResult.channel < 0x80){
					// invalid result.  I.e. the MSb is only set on a valid result.
					continue;
				}

				ADCresult.ADCResult.channel &= 0x7F;				// clear the MSb.  THis marks the result as used and prevents us from re-using it.



				switch(ADCresult.ADCResult.channel){
					case ADC_COMMAND_NUMB__IMON_A:	// Command 2 was the cause of this result.  This is from: ADC_CHANNEL__IMON_A.
						powerBuffers.currentBuffer[0][powerBuffers.bufferIndex ] = ((float)ADCresult.ADCResult.result) * (1.8) / ((float)4096);
						break;
					case ADC_COMMAND_NUMB__IMON_B:	// Command 3 was the cause of this result.  This is from: ADC_CHANNEL__IMON_B.
						powerBuffers.currentBuffer[1][powerBuffers.bufferIndex ] = ((float)ADCresult.ADCResult.result) * (1.8) / ((float)4096);
						break;
					case ADC_COMMAND_NUMB__V_NETWORK:	// Command 1 was the cause of this result.  This is from: ADC_CHANNEL__V_NETWORK.
						powerBuffers.voltageBuffer[powerBuffers.bufferIndex ] = ((float)ADCresult.ADCResult.result) * 1.608 * (1.8 * 48.4) / 4096;		// the 1.608 is a physical hardware thing.  It'll be corrected in the next rev.  I has to do with sampling through high resistors.
						if(++(powerBuffers.bufferIndex) >=  POWER_BUFFER_SIZE) {
							powerBuffers.bufferIndex = 0;
							powerBuffers.voltage = (powerBuffers.voltageBuffer[0] + powerBuffers.voltageBuffer[1] + powerBuffers.voltageBuffer[2] + powerBuffers.voltageBuffer[3]) / 4;
							powerBuffers.current[0] = (powerBuffers.currentBuffer[0][0] + powerBuffers.currentBuffer[0][1] + powerBuffers.currentBuffer[0][2] + powerBuffers.currentBuffer[0][3]) / 4;
							powerBuffers.current[1] = (powerBuffers.currentBuffer[1][0] + powerBuffers.currentBuffer[1][1] + powerBuffers.currentBuffer[1][2] + powerBuffers.currentBuffer[1][3]) / 4;
						}
						break;
					default:	// error scenario.  We shouldn't have any other commands.
						continue;
				}
			}
		    dacConfigStruct.referenceVoltageSource = kACMP_VrefSourceVin2;
		    dacConfigStruct.DACValue = CONVERT_CURRENT_TO_ACMP_SETTING(downstreamCurrentLimit);  //0x7FU; /* Half of referene voltage. */
		    dacConfigStruct.workMode = kACMP_DACWorkLowSpeedMode;
		    ACMP_SetDACConfig(IMONA_ACMP_BASEADDR, &dacConfigStruct);
		    ACMP_SetDACConfig(IMONB_ACMP_BASEADDR, &dacConfigStruct);



#if(0)
			float temp[5];
			int32_t sourcingPowerPort = 0, firstSampleIndex = 0;
			memset(temp,0,sizeof(temp));

			firstSampleIndex = powerBuffers.bufferIndex +1;
			if(firstSampleIndex >= POWER_BUFFER_SIZE) firstSampleIndex = 0;

			for(i = 0; i < POWER_BUFFER_SIZE; i ++){
				temp[0] += powerBuffers.currentBuffer[0][i];
				temp[1] += powerBuffers.currentBuffer[1][i];
				temp[2] += powerBuffers.voltageBuffer[i];
				powerBuffers.powerBuffer[0][i] = powerBuffers.currentBuffer[0][i] * powerBuffers.voltageBuffer[i];
				temp[3] += powerBuffers.powerBuffer[0][i];
				powerBuffers.powerBuffer[1][i] = powerBuffers.currentBuffer[1][i] * powerBuffers.voltageBuffer[i];
				temp[4] += powerBuffers.powerBuffer[1][i];
			}

			powerBuffers.current[0] = (temp[0] / POWER_BUFFER_SIZE);
			powerBuffers.current[1] = (temp[1] / POWER_BUFFER_SIZE);
			powerBuffers.voltage 	= (temp[2] / POWER_BUFFER_SIZE);

			powerBuffers.sourcingPower[0] = powerBuffers.current[0] * powerBuffers.voltage;
			powerBuffers.sourcingPower[1] = powerBuffers.current[1] * powerBuffers.voltage;

			if(powerBuffers.current[0] > powerBuffers.current[1]) sourcingPowerPort = 0; else sourcingPowerPort = 1;

			powerBuffers.dIdt[0] = calculateSlope((float *) &(powerBuffers.currentBuffer[0]), CURRENT_SAMPLING_PERIOD_float, POWER_BUFFER_SIZE, firstSampleIndex);
			powerBuffers.dIdt[1] = calculateSlope((float *) &(powerBuffers.currentBuffer[1]), CURRENT_SAMPLING_PERIOD_float, POWER_BUFFER_SIZE, firstSampleIndex);
			powerBuffers.dVdt 	= calculateSlope((float *)  &powerBuffers.voltageBuffer, CURRENT_SAMPLING_PERIOD_float, POWER_BUFFER_SIZE, firstSampleIndex);

			powerBuffers.dPdt = calculateSlope((float *) &(powerBuffers.powerBuffer[sourcingPowerPort]), CURRENT_SAMPLING_PERIOD_float, POWER_BUFFER_SIZE, firstSampleIndex);


	#define dIdt_DEBOUNCE_MAX 		5
	#define dPdt_DEBOUNCE_MAX		10
	#define CURRENT_DEBOUNCE_MAX	3
	#define SOURCING_POWER_MAX		100		// max power device can source (in Amps).
	//		DIAG_E8_OFF;
			switch(powerState){

				case powerState_init:
					powerStateNext = powerState_fusesOff;
					EFUSE_A_SHUTDOWN_ASSERT;
					EFUSE_B_SHUTDOWN_ASSERT;
					power_flags |= POWERFLAG_SIDEA_EFUSE_SHUTDOWN;
					power_flags |= POWERFLAG_SIDEB_EFUSE_SHUTDOWN;

				break;
				case powerState_fusesOff:
					EFUSE_A_SHUTDOWN_ASSERT;
					EFUSE_B_SHUTDOWN_ASSERT;
					power_flags |= POWERFLAG_SIDEA_EFUSE_SHUTDOWN;
					power_flags |= POWERFLAG_SIDEB_EFUSE_SHUTDOWN;
					if(powerStateTimer_mS > 2000){
						if(powerBuffers.voltage > 20){
							powerStateNext = powerState_sourcingStartup;
						}
						else {
							powerStateNext = powerState_lowVin;
						}
					}
				break;
				case powerState_lowVin:
					EFUSE_A_SHUTDOWN_ASSERT;
					EFUSE_B_SHUTDOWN_ASSERT;
					power_flags |= POWERFLAG_SIDEA_EFUSE_SHUTDOWN;
					power_flags |= POWERFLAG_SIDEB_EFUSE_SHUTDOWN;

					if(powerStateTimer_mS > 2000){
						if(powerBuffers.voltage > 20){
							powerStateNext = powerState_sourcingStartup;

						}
					}

				break;
				case powerState_sourcingStartup:
					if(powerStatePrev != powerState){

						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
						PRINTF("Sourcing Power - Startup\r\n");
						xSemaphoreGive( xPrintMutex );

					}
					if(powerStateTimer_mS > 2000){
						powerStateNext = powerState_sourcing;
						changePIT_Period_uS(SAMPLING_PERIOD_uS);
					}
					if(powerStateTimer_mS > 500){
						EFUSE_A_SHUTDOWN_UN_ASSERT;
						EFUSE_B_SHUTDOWN_UN_ASSERT;
						power_flags &= ~POWERFLAG_SIDEA_EFUSE_SHUTDOWN;
						power_flags &= ~POWERFLAG_SIDEB_EFUSE_SHUTDOWN;

						if(powerBuffers.dIdt[sourcingPowerPort] > 0.5){
							dIdt_debounce++;
							if(dIdt_debounce >= dIdt_DEBOUNCE_MAX){
								powerStateNext = powerState_fault;
							}
						}else{
							dIdt_debounce = 0;
						}
						if(powerBuffers.dPdt > 2.5){
							dPdt_debounce++;
							if(dPdt_debounce >= dPdt_DEBOUNCE_MAX){
								powerStateNext = powerState_fault;
							}
						}else{
							dPdt_debounce = 0;
						}
						if(powerBuffers.sourcingPower[sourcingPowerPort] > SOURCING_POWER_MAX){
							current_debounce++;
							if(current_debounce >= CURRENT_DEBOUNCE_MAX){
								powerStateNext = powerState_fault;
							}
						}else{
							current_debounce = 0;
						}
						if((EFUSE_A_FAULT_INPUT) || (EFUSE_B_FAULT_INPUT)){
							powerStateNext = powerState_fault;
						}
					}
				break;
				case powerState_sourcing:
					EFUSE_A_SHUTDOWN_UN_ASSERT;
					EFUSE_B_SHUTDOWN_UN_ASSERT;
					if(powerStatePrev != powerState){
						if(sourcingPowerPort){ // side B
							power_flags |= POWERFLAG_SIDEB_POWERSOURCE;

						}
						else{ // side A
							power_flags |= POWERFLAG_SIDEA_POWERSOURCE;
						}
						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
						PRINTF("Sourcing Power\r\n");
						xSemaphoreGive( xPrintMutex );

					}
					if((EFUSE_A_FAULT_INPUT) || (EFUSE_B_FAULT_INPUT)){
						powerStateNext = powerState_fault;
					}


				break;
				case powerState_fault:
					if(powerStatePrev != powerState){
						if(sourcingPowerPort){ // side B
							power_flags |= POWERFLAG_SIDEB_EFUSE_FAULT;

						}
						else{ // side A
							power_flags |= POWERFLAG_SIDEA_EFUSE_FAULT;
						}

						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
						PRINTF("POWER FAULT\r\n");
						xSemaphoreGive( xPrintMutex );
					}
					if(powerStateTimer_mS > 10000){

				//		powerStateNext = powerState_fusesOff;
					}
				break;

			}
			powerStatePrev = powerState;
			if(powerStateNext != powerState){
				powerStateTimer_mS = 0;
				powerState = powerStateNext;
			}

			if((powerState == powerState_init) ||
				(powerState == powerState_fusesOff) ||
				(powerState == powerState_lowVin) ||
				(powerState == powerState_fault))	{
	//disabled.  for now.  just until the bugs are worked out.			EFUSE_A_SHUTDOWN_ASSERT;
	//																	EFUSE_B_SHUTDOWN_ASSERT;

			}else{

			}

			if((powerState == powerState_sourcingStartup))
			{
				if(ADCfastSampleMode != 1){
					changePIT_Period_uS(SAMPLING_PERIOD_STARTUP_uS);
				}
				ADCfastSampleMode = 1;
			}else{
				if(ADCfastSampleMode != 0){
					changePIT_Period_uS(SAMPLING_PERIOD_uS);
				}
				ADCfastSampleMode = 0;
			}



#endif

		}


    }


}



// given a series of samples ('inputBufferSize' number of 'y' values, equally spaced with time 'deltaX').
//
//	y - pointer to series of samples
//	deltaX - time between samples.
//  inputBufferSize - number of samples.
//	firstSampleIndex - use this to indicate where in your buffer the first sample is (i.e. index number).
//					- firstSampleIndex is useful if your samples can appear as: 3,4,5,1,2.  instead of 1,2,3,4,5.
//	RETURNS - the slope of the line of best fit through this series.  Uses least squares method.
float calculateSlope(float *y, float deltaX, int inputBufferSize, int firstSampleIndex){
	float xy = 0;
	float sumX = 0;
	float sumY = 0;
	float x2 = 0;
	float slope = 0;
	float x = 0;
	int i, j;
	j = firstSampleIndex;
	for(i = 0; i< inputBufferSize; i++){
		if(j >= inputBufferSize) j = 0;
		x += deltaX;	// because we're actually given x as the distance between x points... i.e. we're just given 1.  not 1,2,3,4,5,...
		xy += x * y[i];
		x2 += x * x;
		sumX += x;
		sumY += y[i];
		j++;
	}

	slope = (inputBufferSize * xy) - (sumX * sumY);
	slope = slope / ((inputBufferSize * x2) - (sumX * sumX));

	return slope;
}





#define BIQUAD_A0_INDEX 0
#define BIQUAD_A1_INDEX 1
#define BIQUAD_A2_INDEX 2
#define BIQUAD_B1_INDEX 3
#define BIQUAD_B2_INDEX 4
//https://www.earlevel.com/main/2021/09/02/biquad-calculator-v3/
float pirHPF_biquads[5] =
{
	1.0,			// a0		// Just a straight up single pole DC block filter.
	-1.0,		// a1
	0.0,			// a2
	0.995,		// b1
	0.0	// b2
};

#if(0)
{
	0.9780302754084559,			// a0		// 20Hz sampling rate, Fc = 0.1, Q = 0.707, gain = 1dB
	-1.9560605508169118,		// a1
	0.9780302754084559,			// a2
	-1.9555778328194147,		// b1
	0.9565432688144089			// b2
};
#endif



float pirLPF_biquads[5] =
{
	0.15735961447072092,		// a0		// 20Hz sampling rate, Fc = 7Hz, Q = 0.1
	0.31471922894144183,		// a1
	0.15735961447072092,		// a2
	0.23301302379216576,		// b1
	-0.6035745659092822			// b2
};


#if(0)
{
	0.6389171933195068,			// a0		// 20Hz sampling rate, Fc = 8Hz, Q = 0.707, gain = 1dB
	1.2778343866390136,			// a1
	0.6389171933195068,			// a2
	1.1429298210046333,			// b1
	0.41273895227339386			// b2
};
#endif

//#define MAX_PIR_PLOTTING_VALUES 		40
void PIR_detect(uint32_t instance,  int32_t *pirInputData, uint32_t number_of_points){

	int i;
//	int tempPlotting[MAX_PIR_PLOTTING_VALUES * 5];

//	float *dataPtr;
	for(i = 0; i <number_of_points; i++){

#if(0)
		// prefilter
		pir_smoothing_filter[instance] = pir_smoothing_filter[instance] - (pir_smoothing_filter[instance] / 8) + pirInputData[i];

		// LPF Biqaud
		pir_LPF_last_sample[instance][2] = pir_LPF_last_sample[instance][1];
		pir_LPF_last_sample[instance][1] = pir_LPF_last_sample[instance][0];
		pir_LPF_last_sample[instance][0] = pirInputData[i];					// input to filter.

		pir_LPF_last_values[instance][0] = pir_LPF_last_sample[instance][0] * pirLPF_biquads[BIQUAD_A0_INDEX];
		pir_LPF_last_values[instance][0] += pir_LPF_last_sample[instance][1] * pirLPF_biquads[BIQUAD_A1_INDEX];
		pir_LPF_last_values[instance][0] +=  pir_LPF_last_sample[instance][2] * pirLPF_biquads[BIQUAD_A2_INDEX];
		pir_LPF_last_values[instance][0] +=  pir_LPF_last_values[instance][0] * pirLPF_biquads[BIQUAD_B1_INDEX] ;
		pir_LPF_last_values[instance][0] +=  pir_LPF_last_values[instance][1] * pirLPF_biquads[BIQUAD_B2_INDEX];

		pir_LPF_last_values[instance][2] = pir_LPF_last_values[instance][1];
		pir_LPF_last_values[instance][1] = pir_LPF_last_values[instance][0];


// HPF Biqaud
		pir_HPF_last_sample[instance][2] = pir_HPF_last_sample[instance][1];
		pir_HPF_last_sample[instance][1] = pir_HPF_last_sample[instance][0];
		pir_HPF_last_sample[instance][0] = pir_LPF_last_values[instance][0];	// input to filter.

		pir_HPF_last_values[instance][0] = pir_HPF_last_sample[instance][0] * pirHPF_biquads[BIQUAD_A0_INDEX] +
										  pir_HPF_last_sample[instance][1] * pirHPF_biquads[BIQUAD_A1_INDEX] +
										  pir_HPF_last_sample[instance][2] * pirHPF_biquads[BIQUAD_A2_INDEX] +
										  pir_HPF_last_values[instance][0] * pirHPF_biquads[BIQUAD_B1_INDEX] +
										  pir_HPF_last_values[instance][1] * pirHPF_biquads[BIQUAD_B2_INDEX];

		pir_HPF_last_values[instance][2] = pir_HPF_last_values[instance][1];
		pir_HPF_last_values[instance][1] = pir_HPF_last_values[instance][0];

		float absValue;
		absValue = fabs(pir_HPF_last_values[instance][0]);
#endif
		float absValue;
		absValue = abs(pirInputData[i]);

		        //compute envelope
        if(absValue / 2 > (pir_envelope_vals[instance]))		{
			pir_envelope_vals[instance] += ((absValue / 2) - pir_envelope_vals[instance]);
		}  //rising = fast filter
        else										{
			pir_envelope_vals[instance] += ((absValue / 2) - pir_envelope_vals[instance]);  //     / 256;
		}  //falling = slow filter

				// accumulate
		pir_accumulator[instance] = pir_envelope_vals[instance] ;

		// boxcar
		pir_output_boxcar[instance][pir_bc_head[instance]] = (uint32_t)pir_accumulator[instance];
//		doppler_output_boxcar[doppler_bc_head] /= 128;

		pir_output[instance] += pir_output_boxcar[instance][pir_bc_head[instance]];
		pir_output[instance] -= pir_output_boxcar[instance][pir_bc_tail[instance]];
		pir_output[instance] = pir_output[instance];
//	  env_comp = env_comp_routine( output_boxcar[accelId][bc_head[accelId]],plot_temp, number_of_points );

		if(++pir_bc_head[instance] >= PIR_OUTPUT_BOXCAR_LEN){pir_bc_head[instance] =0;}
	  	if(++pir_bc_tail[instance] >= PIR_OUTPUT_BOXCAR_LEN){pir_bc_tail[instance] =0;}


		if(instance == 0){
			if(pir_output[instance] / PIR_OUTPUT_BOXCAR_LEN > configParam.PIR0_Config.threshold) {
				discrete_pir_event_tracker(TAP_EVENT, instance);
			}
			leftPIRMagnitude = pir_output[instance] / PIR_OUTPUT_BOXCAR_LEN;
		}else{
			if(pir_output[instance] / PIR_OUTPUT_BOXCAR_LEN > configParam.PIR1_Config.threshold) {
				discrete_pir_event_tracker(TAP_EVENT, instance);
			}
			rightPIRMagnitude = pir_output[instance] / PIR_OUTPUT_BOXCAR_LEN;
		}



        xSemaphoreTake( xJackMutex, ( TickType_t ) portMAX_DELAY );
        	if(((pir_output[instance] / PIR_OUTPUT_BOXCAR_LEN) / eventThreshold) > 100){
        		pirDataForJack[instance] = 100;
        	}else{
        		pirDataForJack[instance] = ((pir_output[instance] / PIR_OUTPUT_BOXCAR_LEN) );
        	}
        xSemaphoreGive( xJackMutex);



	}


}

#define EVENT_COUNT 		(1)		// count
#define EVENT_WINDOW 	    (5.0)		//seconds
#define ALARM_WINDOW 		(30)		// seconds
#define ALARM_HOLD_TIME 	(5)		// seconds
struct detector pir_sensor_status[2];
uint8_t discrete_pir_event_tracker(uint8_t op_code, int instance)
{
  	uint8_t ret_val = 0, j;
//  	int pirAlrmLED_CTRL_ARRAY[2];
 //	char tempMsg[255];


  switch (op_code){
    case(INIT_EVENT):


	  for(j = 0; j<2;j++){
	      pir_sensor_status[j].alarm = 0;
	      pir_sensor_status[j].alarm_timer = 0;
	      pir_sensor_status[j].event_count = 0;
	      pir_sensor_status[j].event_timer = 0;
	      pir_sensor_status[j].event_timer_reload = TICK_PER_SEC * EVENT_WINDOW;           // EVENT_WINDOW;
	      pir_sensor_status[j].alarm_timer_reload = TICK_PER_SEC * ALARM_WINDOW;                //ALARM_WINDOW;
	      pir_sensor_status[j].alarm_hold_timer_reload = TICK_PER_SEC * ALARM_HOLD_TIME;        //ALARM_HOLD;
	      pir_sensor_status[j].event_count_threshold = EVENT_COUNT;                             //EVENT_COUNT_ALARM;

	      pir_sensor_status[j].liftime_event_count =0;
	      pir_sensor_status[j].liftime_alarm_count =0;
	      pir_sensor_status[j].liftime_event_TO_count =0;
	      pir_sensor_status[j].uptime = 0;

	  	}
    	if(instance == 0){
    		PIR_A_LED_OFF;
    		pir1DataForJack = 0;
    	}else{
    		PIR_B_LED_OFF;
    		pir2DataForJack = 0;
    	}



    break;

    case(TAP_EVENT):
      if(pir_sensor_status[instance].event_timer != 0){
        break;
      }



      //reaching this point indicates a new event
      pir_sensor_status[instance].event_count ++;	pir_sensor_status[instance].liftime_event_count ++;   //increment event counters
      pir_sensor_status[instance].event_timer = pir_sensor_status[instance].event_timer_reload;             //set event window timer
      pir_sensor_status[instance].alarm_timer = pir_sensor_status[instance].alarm_timer_reload;             //set alarm window timer
      ret_val =1;                                                               //new event - transmit to base


      if(pir_sensor_status[instance].event_count == pir_sensor_status[instance].event_count_threshold){     //if the event counter == the event count threshold we have an alarm
             pir_sensor_status[instance].alarm = 1;	pir_sensor_status[instance].liftime_alarm_count ++;   //set alarm state and increment alarm counter
             pir_sensor_status[instance].alarm_timer = pir_sensor_status[instance].alarm_hold_timer_reload; //set alarm window timer to the alarm hold time

				 pirConfidenceForJack = 100;
				 pirConfidenceForLogging = 100;
				 setPIRRProxAlarm(1);

			if(instance == 0){
				PIR_A_LED_ON;
				pir1DataForJack = 100;
			}else{
				PIR_B_LED_ON;
				pir2DataForJack = 100;
			}

      }

      else if(pir_sensor_status[instance].event_count > pir_sensor_status[instance].event_count_threshold){ //if the event counter > the event count threshold we have a the continuation of a already declared alarm
             pir_sensor_status[instance].event_count = pir_sensor_status[instance].event_count_threshold;   //set the event count back to its threshold level so it doesn't increase forever
             pir_sensor_status[instance].alarm_timer = pir_sensor_status[instance].alarm_hold_timer_reload; //reset the alarm hold time
             ret_val =0;                                                        //retriggerign existing alarm - do not transmit to base
      }

    break;

    case(TIMER_TICK_EVENT):     //a timer tick has elapsed

      if(pir_sensor_status[instance].event_timer) {                                           //if the event window timer is set process it
        if(--pir_sensor_status[instance].event_timer == 0){                                   //if the event window timer is expiring

          ret_val =1;                                                           //transmit event mag to base
                                                   //clear peak hold
        }
      }

      if(pir_sensor_status[instance].alarm_timer){                                            //if the alarm window timer is set process it
    	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  //keep the accelerometer awake if there are any events on the books
        if(--pir_sensor_status[instance].alarm_timer ==0){                                    //if the alarm window timer is expiring
          if(pir_sensor_status[instance].alarm == 0){pir_sensor_status[instance].liftime_event_TO_count ++;} //if no alarm was declared then increment a event timeout counter
          pir_sensor_status[instance].alarm = 0;                                              //clear alarm status
          pir_sensor_status[instance].event_count = 0;                                        //clear event count
          ret_val =1;                                                           //alarm or events have cleared - transmit to base
			if(instance == 0){
		        PIR_A_LED_OFF;
				pir1DataForJack = 0;
			}else{
				PIR_B_LED_OFF;
				pir2DataForJack = 0;
			}

        }
      }
      pir_sensor_status[instance].uptime ++;                                                  //increment diagnostic uptime counter
    break;

    default:
    break;

  }

  return ret_val;
 }




 extern struct STR_PIRConfig PIRSettings[2];
 void PYD1598Setup(void){

 	uint32_t i,j;





 	// run the settings through the serial link.
 	PIR1_SL_CLEAR;
 	PIR2_SL_CLEAR;

 	PIR1_MsgWord.SLmsg.threshold = PIRSettings[0].threshold;
 	PIR1_MsgWord.SLmsg.blindTime = PIRSettings[0].blindTime_mS;
 	PIR1_MsgWord.SLmsg.pulseCounter = PIRSettings[0].pulseCount;
 	PIR1_MsgWord.SLmsg.windowTime = PIRSettings[0].windowTime_S;
 	PIR1_MsgWord.SLmsg.operationMode = DIGIPYRO_OPP_MODE;
 	PIR1_MsgWord.SLmsg.signalSource = DIGIPYRO_SIG_SOURCE;
 	PIR1_MsgWord.SLmsg.reserved_0x2 = 0x02;
 	PIR1_MsgWord.SLmsg.HPFcutoff = PIRSettings[0].HPFCutOff;
 	PIR1_MsgWord.SLmsg.reserved_0x0 = 0x00;
 	PIR1_MsgWord.SLmsg.countMode = PIRSettings[0].countMode;

 	// shift left so we can shift out MSB first.
 	PIR1_MsgWord.raw = PIR1_MsgWord.raw << 7;

 	PIR2_MsgWord.SLmsg.threshold = PIRSettings[1].threshold;
 	PIR2_MsgWord.SLmsg.blindTime = PIRSettings[1].blindTime_mS;
 	PIR2_MsgWord.SLmsg.pulseCounter = PIRSettings[1].pulseCount;
 	PIR2_MsgWord.SLmsg.windowTime = PIRSettings[1].windowTime_S;
 	PIR2_MsgWord.SLmsg.operationMode = DIGIPYRO_OPP_MODE;
 	PIR2_MsgWord.SLmsg.signalSource = DIGIPYRO_SIG_SOURCE;
 	PIR2_MsgWord.SLmsg.reserved_0x2 = 0x02;
 	PIR2_MsgWord.SLmsg.HPFcutoff = PIRSettings[1].HPFCutOff;
 	PIR2_MsgWord.SLmsg.reserved_0x0 = 0x00;
 	PIR2_MsgWord.SLmsg.countMode = PIRSettings[1].countMode;

 	PIR2_MsgWord.raw = PIR2_MsgWord.raw << 7;

 	vTaskSuspendAll();	// pause the scheduler, so we don't task swap here.
 	// need to have Direct Link line already low to do this.
 	for(j = 0; j < 25; j++){
 		PIR1_SL_CLEAR;	PIR2_SL_CLEAR;
 		for(i = 0; i< 150; i++){ asm("NOP");	}		// must be between 0.2uS to 2uS.

 		PIR1_SL_SET;	PIR2_SL_SET;
 		for(i = 0; i< 150; i++){ asm("NOP");	}		// must be between 0.2uS to 2uS.

 		if(PIR1_MsgWord.raw & 0x80000000){
 			PIR1_SL_SET;
 		}else{
 			PIR1_SL_CLEAR;
 		}
 		if(PIR2_MsgWord.raw & 0x80000000){
 			PIR2_SL_SET;
 		}else{
 			PIR2_SL_CLEAR;
 		}
 		for(i = 0; i< 20000; i++){ asm("NOP");	}	// must be greater than 80uS, less than 150uS.
 		PIR1_MsgWord.raw = PIR1_MsgWord.raw << 1;
 		PIR2_MsgWord.raw = PIR2_MsgWord.raw << 1;
 	}

 	PIR1_SL_CLEAR;
 	PIR2_SL_CLEAR;
 	xTaskResumeAll();	// Resume the scheduler.

 	vTaskDelay(1000 / portTICK_PERIOD_MS );		// // must be greater than 650uS


 }


 uint32_t getPYD1598Data(int32_t *pir1, int32_t *pir2){

	 uint32_t attempt = 0;
	 uint32_t pirError = 0;

	 do{


		PIR_MsgIndex = 0;
		PIR1_MsgWord.raw = 0;
		PIR2_MsgWord.raw = 0;

		PIR1_DL_SET;
		PIR2_DL_SET;
		PIR1_DL_DIR_OUTPUT;	 	PIR2_DL_DIR_OUTPUT;		//
		vTaskDelay(2 / portTICK_PERIOD_MS );	// this must be longer than Tds (150us).

		// timing is important for the sampling below.  Thus we need to pause the scheduler.
		// the below works with Tbit of about 2uS.  tDL and tDH are jsut under 300nS.  1.5uS for the tBS settling time.
		// 40bits at 2uS makes the suspend time to be about 80uS.

		vTaskSuspendAll();	// pause the scheduler, so we don't task swap here.
		do{

			PIR1_DL_CLEAR;  		PIR2_DL_CLEAR;
			PIR1_DL_DIR_OUTPUT;	 	PIR2_DL_DIR_OUTPUT;		//	1 = output, bit 14.

			for(onChipRamIndex_i = 0; onChipRamIndex_i< 15; onChipRamIndex_i++){ asm("NOP");	}	// between 0.2uS to 2uS.
			PIR1_DL_SET;			PIR2_DL_SET;

			for(onChipRamIndex_i = 0; onChipRamIndex_i< 15; onChipRamIndex_i++){ asm("NOP");	}	// between 0.2uS to 2uS.
			PIR1_DL_DIR_INPUT;		PIR2_DL_DIR_INPUT;		//	0 = input,
			for(onChipRamIndex_i = 0; onChipRamIndex_i< 60; onChipRamIndex_i++){ asm("NOP");	}	// must be less than 22uS.  But depends on fall time of bit.

			PIR1_MsgWord.raw = PIR1_MsgWord.raw * 2;		// left shift.
			PIR2_MsgWord.raw = PIR2_MsgWord.raw * 2;		// left shift.
			if(PIR1_DL_DATA_IN){
				PIR1_MsgWord.raw++;					// set the LSB.
			}
			if(PIR2_DL_DATA_IN){
				PIR2_MsgWord.raw++;					// set the LSB.
			}

			PIR_MsgIndex++;						// increment our index.


			if(PIR_MsgIndex >= 40){
				PIR1_DL_CLEAR;			PIR2_DL_CLEAR;	// finish on a 0.  For Tra and Tup.
				PIR1_DL_DIR_OUTPUT;		PIR2_DL_DIR_OUTPUT;	//	1 = output, bit 14.
			}
		}while((PIR_MsgIndex < 40));	// best ERR is likely a compare on our Config data vs the last 25bits here (not implemented yet).
		xTaskResumeAll();	// Resume the scheduler.

		vTaskDelay(5 / portTICK_PERIOD_MS );	// this must be longer than Tra + Tup (160us + 1250us).
		PIR1_DL_DIR_OUTPUT;			PIR2_DL_DIR_OUTPUT;//	1 = output, bit 14.
		PIR1_DL_CLEAR;				PIR2_DL_CLEAR;

		pirError = 0;


		if((PIR1_MsgWord.DLmsg.reserved_0x2 != 0x02)){
			// this appears to happen occasionaly.  I believe it's when we get an ISR at a bad time and it takes to long (shame on us for sitting in an ISR too long).
			// however, the symptom often appears that we're simply out by a bit.  I.e. a left bitshift on our RAW will correct this.
			PIR1_MsgWord.raw = PIR1_MsgWord.raw << 1;
			// now verify.
			if(!(	(PIR1_MsgWord.DLmsg.reserved_0x2 == 0x02) &&
					(PIR1_MsgWord.DLmsg.reserved_0x0 == 0x00) &&
					(PIR1_MsgWord.DLmsg.threshold == PIRSettings[0].threshold) &&
					(PIR1_MsgWord.DLmsg.blindTime == PIRSettings[0].blindTime_mS) &&
					(PIR1_MsgWord.DLmsg.pulseCounter == PIRSettings[0].pulseCount) &&
					(PIR1_MsgWord.DLmsg.windowTime == PIRSettings[0].windowTime_S) &&
					(PIR1_MsgWord.DLmsg.HPFcutoff == PIRSettings[0].HPFCutOff) &&
					(PIR1_MsgWord.DLmsg.countMode == PIRSettings[0].countMode)
					))
			{
				// didn't recover
				pirError = pirError - 1;
			}
		}
		if((PIR2_MsgWord.DLmsg.reserved_0x2 != 0x02)){
			// this appears to happen occasionaly.  I believe it's when we get an ISR at a bad time and it takes to long (shame on us for sitting in an ISR too long).
			// however, the symptom often appears that we're simply out by a bit.  I.e. a left bitshift on our RAW will correct this.
			PIR2_MsgWord.raw = PIR2_MsgWord.raw << 1;
			// now verify.
			if(!(	(PIR2_MsgWord.DLmsg.reserved_0x2 == 0x02) &&
					(PIR2_MsgWord.DLmsg.reserved_0x0 == 0x00) &&
					(PIR2_MsgWord.DLmsg.threshold == PIRSettings[1].threshold) &&
					(PIR2_MsgWord.DLmsg.blindTime == PIRSettings[1].blindTime_mS) &&
					(PIR2_MsgWord.DLmsg.pulseCounter == PIRSettings[1].pulseCount) &&
					(PIR2_MsgWord.DLmsg.windowTime == PIRSettings[1].windowTime_S) &&
					(PIR2_MsgWord.DLmsg.HPFcutoff == PIRSettings[1].HPFCutOff) &&
					(PIR2_MsgWord.DLmsg.countMode == PIRSettings[1].countMode)
					))
			{
				// didn't recover
				pirError = pirError - 2;
			}
		}
		attempt++;
	 }while((attempt < 4) && (pirError != 0));
 	//	we recovered, and we can carry on like nothing happened.

	if(pirError != 0){
		return pirError;
	}


 	if(PIR1_MsgWord.DLmsg.signalSource == 0x00) {	// output is after BPF (int)
 		*pir1 = (int16_t)PIR1_MsgWord.DLmsg.adcCounts ;
 	}else if(PIR1_MsgWord.DLmsg.signalSource == 0x01) {	// output is after LPF (unsigned int)
 		*pir1 = (uint16_t)PIR1_MsgWord.DLmsg_uint.adcCounts ;
 	}else{	// output is temperature sensor.	(unsigned it)
 		*pir1 = (uint16_t)PIR1_MsgWord.DLmsg_uint.adcCounts ;
 	}

 	if(PIR2_MsgWord.DLmsg.signalSource == 0x00) {	// output is after BPF (int)
 		*pir2 = (int16_t)PIR2_MsgWord.DLmsg.adcCounts ;
 	}else if(PIR2_MsgWord.DLmsg.signalSource == 0x01) {	// output is after LPF (unsigned int)
 		*pir2 = (uint16_t)PIR2_MsgWord.DLmsg_uint.adcCounts ;
 	}else{	// output is temperature sensor.	(unsigned it)
 		*pir2 = (uint16_t)PIR2_MsgWord.DLmsg_uint.adcCounts ;
 	}

 	if((PIR1_MsgWord.DLmsg.outOfRange == 0) || (PIR2_MsgWord.DLmsg.outOfRange == 0)){
 			// PIR was reset due to an out of range value on its ADC
 		return 1;
 	}

 	return 0;	// normal.
 }







