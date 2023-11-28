/*
 * app_audio.c
 *
 *  Created on: Apr. 19, 2021
 *      Author: stippett
 */
#include <string.h>

#include "app_audio.h"
#include "fsl_pdm.h"
#include "fsl_pdm_edma.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_gpt.h"
#include "fsl_anatop_ai.h"

#include "app_shared.h"
//#include "app_shield.h"

#include "core/net.h"

//#include "McuWait.h"
//#include "McuRTOS.h"
#include "peripherals.h"
#include "app_logging.h"
#include "app_config.h"
#include "app_jack.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define DEMO_PDM                      PDM
#define PDM_ERROR_IRQn           PDM_ERROR_IRQn
#define PDM_ERROR_IRQHandler     PDM_ERROR_IRQHandler
#define PDM_CLK_FREQ             CLOCK_GetRootClockFreq(kCLOCK_Root_Mic)//32768000//24576000//
#define PDM_FIFO_WATERMARK       (4)
#define PDM_QUALITY_MODE         kPDM_QualityModeHigh
#define PDM_CIC_OVERSAMPLE_RATE  (8U)

#define PDM_SAMPLE_CLOCK_RATE    (3268000) /* 3.268MHZ */
//#define DEMO_EDMA               DMA0
//#define DEMO_DMAMUX             DMAMUX0

#define DEMO_PDM_REQUEST_SOURCE       kDmaRequestMuxPdm

#define BUFFER_SIZE_PDM   			(CHUNK_SIZE*NUM_MICS)
#define CHUNK_SIZE   				(6400)	// unit here is bytes.  4 bytes per sample.  with 100mS interrupt period, 1600 samples at 16kHz audio sampling rate. 1600 x 4 = 6400
#define BUFFER_NUMBER 				(2)		// 2 buffers allows us to ping pong them.
#define BYTES_PER_SAMPLE			4
#define CHUNKS_TO_SEND				125 * NUM_MICS // with 3 mics this should be a half second of audio data
#define HEADER_SIZE					(8+6) // 8 bytes of """demarkation""", 1 byte audio channel, 1 byte sequence number, 4 bytes uint32_t microseconds since last udp sync msg
#define AUDIO_DATA_SETTING				(2) // 0 is un-interleaved data, 1 is predefined audio (not interleaved), 2 is as-is interleaved data
#define SYNC_GPT_CLK_FREQ CLOCK_GetFreq(kCLOCK_Osc24M)
#define AUD_PLL_MFN_MAX 100000
#define AUD_PLL_MFN_MIN 0
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void pdmCallback(PDM_Type *base, pdm_edma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_QUICKACCESS_SECTION_DATA_ALIGN(pdm_edma_handle_t s_pdmRxHandle_0, 4);
AT_QUICKACCESS_SECTION_DATA_ALIGN(edma_handle_t s_pdmDmaHandle_0, 4);
//pdm_edma_handle_t s_pdmRxHandle_0;


AT_QUICKACCESS_SECTION_DATA_ALIGN(edma_tcd_t s_edmaTcd_0[2], 32U);
//
//AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t s_edmaTcd_0[2], 32U);
//AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t s_edmaTcd_1[2], 32U);
//AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_buffer[BUFFER_SIZE_PDM * BUFFER_NUMBER], 4);
AT_QUICKACCESS_SECTION_DATA_ALIGN(static uint8_t s_buffer[BUFFER_SIZE_PDM * BUFFER_NUMBER], 4);
__BSS(BOARD_SDRAM) uint8_t audio_ch_data[CHUNKS_TO_SEND][CHUNK_SIZE];
static volatile bool s_lowFreqFlag   = false;
static volatile bool s_fifoErrorFlag = false;

pdm_edma_transfer_t pdmXfer[2] =
    {
        {
        .data  = s_buffer,
        .dataSize = BUFFER_SIZE_PDM,
        .linkTransfer = &pdmXfer[1],
       },

        {
        .data  = &s_buffer[BUFFER_SIZE_PDM],
        .dataSize = BUFFER_SIZE_PDM,
        .linkTransfer = &pdmXfer[0]
        },
    };

static volatile uint32_t s_bufferValidBlock = BUFFER_NUMBER;
static volatile uint32_t s_readIndex        = 0U;
static volatile uint32_t s_writeIndex       = 0U;
static const pdm_config_t pdmConfig         = {
		.enableDoze        = false,
		.fifoWatermark     = PDM_FIFO_WATERMARK,
		.qualityMode       = PDM_QUALITY_MODE,
		.cicOverSampleRate = PDM_CIC_OVERSAMPLE_RATE,
};


/* Stores the handle of the task that will be notified when the
transmission is complete. */
static TaskHandle_t xTaskToNotify = NULL;

/* The index within the target task's array of task notifications
to use. */
//static const UBaseType_t xArrayIndex = 0;


uint32_t udp_sync_us = 0;
uint32_t timestamp_us = 0; // for capturing udp_sync_us at the time PDM data is ready to be sent
uint32_t udp_master_time_us = 0;



static volatile uint32_t samples_collected = 0;


int32_t	hfmemsErrorFlag = 0;
int32_t	hfmemsErrorFlagPrev = 0;

int32_t	getHFMEMSError(void){
	int32_t err;
	err = hfmemsErrorFlag;
	if (hfmemsErrorFlag != 0) {
				return 1;
			} // the above is temp while we aren't caring about specific errors
	return err;
}

/*******************************************************************************
 * Code
 ******************************************************************************/
void GPT3_IRQHandler(void)
{
	/* Clear interrupt flag.*/
	GPT_ClearStatusFlags(GPT3, kGPT_OutputCompare1Flag);
	udp_sync_us++;

	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F, Cortex-M7, Cortex-M7F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U || __CORTEX_M == 7U)
	__DSB();
#endif
}
static void AppConfigureTimerForUDPSync(void) {

	/* Enable GPT Output Compare1 interrupt */
	GPT_EnableInterrupts(GPT3, kGPT_OutputCompare1InterruptEnable);

	/* Enable at the Interrupt and start timer */
	EnableIRQ(GPT3_IRQn);
	GPT_StartTimer(GPT3);
}
volatile uint8_t *pingPong_audioBufferDataPtr = NULL;		// this points to new data in the audio Sampling buffer
uint32_t	dataFault = 0;
void pdmCallback(PDM_Type *base, pdm_edma_handle_t *handle, status_t status, void *userData)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	uint32_t sample_set_index;
//	uint32_t sample_index;
	//memcpy(&timestamp_us, &udp_sync_us, sizeof(udp_sync_us));
	samples_collected += BUFFER_SIZE_PDM/(BYTES_PER_SAMPLE*NUM_MICS); // 4 bytes per sample and 3 mics have samples in buffer
	if(s_readIndex == BUFFER_NUMBER){
		s_readIndex = 0U;

	}
	if(s_writeIndex == CHUNKS_TO_SEND) s_writeIndex = 0;
	//memcpy(&audio_ch_data[s_writeIndex], &s_buffer[BUFFER_SIZE * s_readIndex], BUFFER_SIZE);
//	pingPong_audioBufferDataPtr = (uint8_t *)handle->tcd->DADDR;		// this will be the location where data was just coppied to.

	if(pingPong_audioBufferDataPtr != NULL){
		DIAG_E8_ON;		// for a breakpoint.
		dataFault = 1;
	}else{
		dataFault = 0;
	}


	pingPong_audioBufferDataPtr = (uint8_t *)DMA0->TCD[s_pdmRxHandle_0.tcdUser].DADDR;
#if(0)
	if(pingPong_audioBufferDataPtr < &s_buffer[BUFFER_SIZE_PDM]){
		pingPong_audioBufferDataPtr = &s_buffer[0];
		DIAG_E8_ON;
	}else if (pingPong_audioBufferDataPtr >= &s_buffer[BUFFER_SIZE_PDM]) {
		pingPong_audioBufferDataPtr = &s_buffer[BUFFER_SIZE_PDM];
		DIAG_E8_OFF;
	}
#endif

#if(0)	// copy the relevant channel data.				-- T.S. I pulled this from the ISR, because it's way too much stuff to be doing in an ISR.
	for( sample_set_index = 0;
			sample_set_index < BUFFER_SIZE_PDM;
			sample_set_index+= (BYTES_PER_SAMPLE*NUM_MICS)
	){
		// sample_set_index defines the index within the buffer that is the start of a new set of channels' samples
		for( sample_index = 0;
				sample_index < (BYTES_PER_SAMPLE*NUM_MICS);
				sample_index+= BYTES_PER_SAMPLE
		){
			if(sample_index == 0*BYTES_PER_SAMPLE ){ // only copy the first and last channel samples as that's the only data we currently want
				memcpy(&audio_ch_data[s_writeIndex][sample_set_index/NUM_MICS],
						&s_buffer[(BUFFER_SIZE_PDM * s_readIndex)+sample_set_index+sample_index], BYTES_PER_SAMPLE);
			}
#if NUM_MICS >= 2
			else if(sample_index == 1*BYTES_PER_SAMPLE){ // the second channel
				memcpy(&audio_ch_data[s_writeIndex+1][sample_set_index/NUM_MICS],
						&s_buffer[(BUFFER_SIZE_PDM * s_readIndex)+sample_set_index+sample_index], BYTES_PER_SAMPLE);
			}
#endif
#if NUM_MICS >= 3
			else if(sample_index == 2*BYTES_PER_SAMPLE){ // the third channel
				memcpy(&audio_ch_data[s_writeIndex+2][sample_set_index/NUM_MICS],
						&s_buffer[(BUFFER_SIZE_PDM * s_readIndex)+sample_set_index+sample_index], BYTES_PER_SAMPLE);
			}
#endif
#if NUM_MICS >= 4
			else if(sample_index == 3*BYTES_PER_SAMPLE){ // the fourth channel
				memcpy(&audio_ch_data[s_writeIndex+3][sample_set_index/NUM_MICS],
						&s_buffer[(BUFFER_SIZE_PDM * s_readIndex)+sample_set_index+sample_index], BYTES_PER_SAMPLE);
			}
#endif

		}
	}
	memset(&s_buffer[BUFFER_SIZE_PDM * s_readIndex], 0,BUFFER_SIZE_PDM);
	s_readIndex++;
	s_writeIndex+=NUM_MICS; // increment by 3 because we are filling three chunks within audio_ch_data
	/* Notify the task that the transmission is complete. */
#endif
	if(s_writeIndex == CHUNKS_TO_SEND){
		timestamp_us = udp_sync_us + udp_master_time_us;
	}
//	vTaskNotifyGiveIndexedFromISR( xTaskToNotify,
//			xArrayIndex,
//			&xHigherPriorityTaskWoken );
	/* If xHigherPriorityTaskWoken is now set to pdTRUE then a
	    context switch should be performed to ensure the interrupt
	    returns directly to the highest priority task.  The macro used
	    for this purpose is dependent on the port in use and may be
	    called portEND_SWITCHING_ISR(). */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void PDM_ERROR_IRQHandler(void)
{
	uint32_t status = 0U;
	if (PDM_GetStatus(PDM) & PDM_STAT_LOWFREQF_MASK)
	{
		PDM_ClearStatus(PDM, PDM_STAT_LOWFREQF_MASK);
		s_lowFreqFlag = true;
	}

	status = PDM_GetFifoStatus(PDM);
	if (status)
	{
		PDM_ClearFIFOStatus(PDM, status);
		s_fifoErrorFlag = true;
	}

#if defined(FSL_FEATURE_PDM_HAS_RANGE_CTRL) && FSL_FEATURE_PDM_HAS_RANGE_CTRL
	status = PDM_GetRangeStatus(PDM);
	if (status)
	{
		PDM_ClearRangeStatus(PDM, status);
	}
#else
	status = PDM_GetOutputStatus(PDM);
	if (status)
	{
		PDM_ClearOutputStatus(PDM, status);
	}
#endif

	hfmemsErrorFlagPrev = hfmemsErrorFlag;
	hfmemsErrorFlag |= status; // don't care about specifics right now though
	if (hfmemsErrorFlag != hfmemsErrorFlagPrev){
		// change in error state
		// so flag to the jack task that there's a change
		diagnostic_change_flag |= 1;
		if(hfmemsErrorFlag != 0){
			sys_stats.sensorstat.faultHFMEMS_count++;
		}
	}
	__DSB();
}
static void adjust_audio_pll_mfn(int32_t pllMfnAdjustVal){
	int32_t current_aud_pll_numerator = ANATOP_AI_Read(kAI_Itf_Audio, kAI_PLLAUDIO_CTRL2);


	if ((current_aud_pll_numerator + pllMfnAdjustVal) < AUD_PLL_MFN_MIN){
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("	Sync task: previous PLL MFN value of %d, adjusting to minimum of %d\r\n", current_aud_pll_numerator, AUD_PLL_MFN_MIN);
		xSemaphoreGive( xPrintMutex );
		ANATOP_AI_Write(kAI_Itf_Audio, kAI_PLLAUDIO_CTRL2, (uint32_t) AUD_PLL_MFN_MIN);
	}
	else if ((current_aud_pll_numerator + pllMfnAdjustVal) > AUD_PLL_MFN_MAX){
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("	Sync task: previous PLL MFN value of %d, adjusting to maximum of %d\r\n", current_aud_pll_numerator, AUD_PLL_MFN_MAX);
		xSemaphoreGive( xPrintMutex );
		ANATOP_AI_Write(kAI_Itf_Audio, kAI_PLLAUDIO_CTRL2, (uint32_t) AUD_PLL_MFN_MAX);
	}
	else{
		if (pllMfnAdjustVal < 0){
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("	Sync task: previous PLL MFN value of %d, adjusting by -%d\r\n", current_aud_pll_numerator, pllMfnAdjustVal);
			xSemaphoreGive( xPrintMutex );
		}
		else{
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("	Sync task: previous PLL MFN value of %d, adjusting by %d\r\n", current_aud_pll_numerator, pllMfnAdjustVal);
			xSemaphoreGive( xPrintMutex );
		}

		ANATOP_AI_Write(kAI_Itf_Audio, kAI_PLLAUDIO_CTRL2, (uint32_t) (current_aud_pll_numerator + pllMfnAdjustVal));
	}
}
__attribute__ ((optimize("-O0")))
static void tune_sample_rate(uint32_t time_us, uint32_t num_samples, uint32_t desired_rate){
	/*
	 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
	 *                              = 24 * (32 + 92/100)
	 *                              = 790.08 MHz
	 */
//		const clock_audio_pll_config_t audioPllConfig = {
//				.loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
//				.postDivider = 0,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
//				.numerator   = 92,  /* 30 bit numerator of fractional loop divider. */
//				.denominator = 100, /* 30 bit denominator of fractional loop divider */
//		};
	uint32_t rate_diffx10;
	//uint32_t pllMfnAdjustVal = 0;
	//float desired_sample_period_us = (1 / (float) desired_rate) * 1000000;
	float current_period = (float) time_us / (float) num_samples;
	float current_rate_f = (1/ current_period);
	uint32_t current_rate = (uint32_t) (current_rate_f * 1000000);
	uint32_t current_ratex10 = (uint32_t) (current_rate_f * 10000000);

	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
	PRINTF("	Sync task: the estimated audio sample rate is %uHz vs the desired rate of %uHz.\r\n", current_rate, desired_rate);
	PRINTF("	Sync task: PDM_ROOT_CLOCK is %uHz, CLKDIV is %u, and PDM_CLOCK should be around %uHz.\r\n", CLOCK_GetRootClockFreq(kCLOCK_Root_Mic),(PDM->CTRL_2 & (PDM_CTRL_2_CLKDIV_MASK)>> PDM_CTRL_2_CLKDIV_SHIFT), CLOCK_GetRootClockFreq(kCLOCK_Root_Mic)/(PDM->CTRL_2 & (PDM_CTRL_2_CLKDIV_MASK)));

	xSemaphoreGive( xPrintMutex );
	// below pll adjust values are based on a denominator of 100000
	if (current_rate > desired_rate){
		// too fast
		rate_diffx10 = current_ratex10 - (desired_rate*10);
		if (rate_diffx10 > 10000) adjust_audio_pll_mfn(-10000);
		else if (rate_diffx10 > 5000) adjust_audio_pll_mfn(-5000);
		else if (rate_diffx10 > 1000) adjust_audio_pll_mfn(-1000);
		else if (rate_diffx10 > 500) adjust_audio_pll_mfn(-500);
		else if (rate_diffx10 > 100) adjust_audio_pll_mfn(-100);
		else if (rate_diffx10 > 50) adjust_audio_pll_mfn(-50);
		else if (rate_diffx10 > 10) adjust_audio_pll_mfn(-10);
		else if (rate_diffx10 > 5) adjust_audio_pll_mfn(-5);
		else if (rate_diffx10 > 1) adjust_audio_pll_mfn(-1);
	}
	else if (current_rate < desired_rate){
		// too slow
		rate_diffx10 = (desired_rate*10) - current_ratex10;
		if (rate_diffx10 > 10000) adjust_audio_pll_mfn(10000);
		else if (rate_diffx10 > 5000) adjust_audio_pll_mfn(5000);
		else if (rate_diffx10 > 1000) adjust_audio_pll_mfn(1000);
		else if (rate_diffx10 > 500) adjust_audio_pll_mfn(500);
		else if (rate_diffx10 > 100) adjust_audio_pll_mfn(100);
		else if (rate_diffx10 > 50) adjust_audio_pll_mfn(50);
		else if (rate_diffx10 > 10) adjust_audio_pll_mfn(10);
		else if (rate_diffx10 > 5) adjust_audio_pll_mfn(5);
		else if (rate_diffx10 > 1) adjust_audio_pll_mfn(1);
	}
	return;
}


struct STR_HFMEMSConfig	HFMEMSConfig;

void setHFMEMSConfig(struct STR_HFMEMSConfig *newConfig){
	memcpy(&HFMEMSConfig, newConfig, sizeof(newConfig));
}
void getHFMEMSConfig(struct STR_HFMEMSConfig *newConfig){
	memcpy(newConfig, &HFMEMSConfig, sizeof(newConfig));
}
uint32_t HFMEMSMagnitude = 0;
uint32_t getHFMEMSMagnitude(void){
	uint32_t rv;

		rv = HFMEMSMagnitude;
		HFMEMSMagnitude = 0;

	return rv;
}
int audio_init(void){

	clock_root_config_t rootCfg = {0};
	/*
	 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
	 *                              = 24 * (32 + 92/100)
	 *                              = 790.08 MHz
	 */
		const clock_audio_pll_config_t audioPllConfig = {
				.loopDivider = 28,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
				.postDivider = 0,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
				.numerator   = 59880,  /* 30 bit numerator of fractional loop divider. */
				.denominator = 100000, /* 30 bit denominator of fractional loop divider */
		};
	    rootCfg.mux = kCLOCK_MIC_ClockRoot_MuxAudioPllOut;
	    rootCfg.div = 9;
		static const pdm_channel_config_t channelConfig = {
				.cutOffFreq = kPDM_DcRemoverCutOff152Hz,
				.gain       = kPDM_DfOutputGain7,
		};
//	edma_config_t dmaConfig = {0};

	CLOCK_InitAudioPll(&audioPllConfig);
	CLOCK_SetRootClock(kCLOCK_Root_Mic, &rootCfg);

	HFMEMSConfig.HPFSetting = 0;
	HFMEMSConfig.LPFSetting = 0;
	HFMEMSConfig.threshold = 0;
	HFMEMSConfig.LRInterior = 0;
	HFMEMSConfig.privacy = 0;






#if defined(FSL_FEATURE_EDMA_HAS_CHANNEL_MUX) && FSL_FEATURE_EDMA_HAS_CHANNEL_MUX
	EDMA_SetChannelMux(DEMO_EDMA, DEMO_EDMA_CHANNEL, DEMO_PDM_EDMA_CHANNEL);
#endif
	/* Setup pdm */
	PDM_Init(PDM, &pdmConfig);

    PDM_TransferCreateHandleEDMA(PDM, &s_pdmRxHandle_0, pdmCallback, NULL, &s_pdmDmaHandle_0);
    PDM_TransferInstallEDMATCDMemory(&s_pdmRxHandle_0, s_edmaTcd_0, 2);

    			// you need to make sure the last function call here is the highest channel number //
    			// otherwise s_pdmRxHandle_0->endChannel will be incorrect.  T.S. 2022-10-19	  //
    PDM_TransferSetChannelConfigEDMA(PDM, &s_pdmRxHandle_0, PDM_ENABLE_CHANNEL0_RIGHT, &channelConfig);
	PDM_TransferSetChannelConfigEDMA(PDM, &s_pdmRxHandle_0, PDM_ENABLE_CHANNEL0_LEFT, &channelConfig);
	PDM_TransferSetChannelConfigEDMA(PDM, &s_pdmRxHandle_0, PDM_ENABLE_CHANNEL1_RIGHT, &channelConfig);
//	PDM_TransferSetChannelConfigEDMA(PDM, &s_pdmRxHandle_0, PDM_ENABLE_CHANNEL1_LEFT, &channelConfig);

	if (PDM_SetSampleRateConfig(PDM, PDM_CLK_FREQ, AUDIO_SAMPLE_RATE) != kStatus_Success)
	{
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("PDM configure sample rate failed.\r\n");
		xSemaphoreGive( xPrintMutex );

		return -1;
	}


	PDM_EnableInterrupts(PDM, kPDM_ErrorInterruptEnable);
	EnableIRQ(PDM_ERROR_IRQn);



#if(0)
 NO! these QSEL CICOSR are used in checking sample rate.  So even if you were to do this, you need
	//			to check it BEFORE you call setSampleRateConfig.  It's best to let PDM_Init handle this.  T.S. 2022-10-15
//	PDM->CTRL_2 = (PDM->CTRL_2 & (~(PDM_CTRL_2_CICOSR_MASK | PDM_CTRL_2_QSEL_MASK))) |
//			PDM_CTRL_2_CICOSR(7) | PDM_CTRL_2_QSEL(kPDM_QualityModeHigh);
//	PDM->CTRL_2 = (PDM->CTRL_2 & (~PDM_CTRL_2_CLKDIV_MASK)) | PDM_CTRL_2_CLKDIV(66);
#endif
	PDM_Reset(PDM);

	PDM_TransferReceiveEDMA(PDM, &s_pdmRxHandle_0, pdmXfer);
//	uint32_t i;
//	for(i = 0; i < 100; i++){
//		SDK_DelayAtLeastUs(500000, CLOCK_GetFreq(kCLOCK_CpuClk));		// debug... just so I can check if it's working.
//
//	}

	EDMA_TcdDisableInterrupts((edma_tcd_t *)&s_pdmRxHandle_0, (uint32_t)kEDMA_MajorInterruptEnable);
	return 0;
}



#if(0)
void udp_sync_task(void *pvParameters){
	struct netconn *conn;
	struct netbuf *net_buf;
	char buffer[1024];
	uint32_t udpMessageNum, timerCount, prevUdpMessageNum;
	uint8_t intended_node_num, node_num_byte;
	//int32_t current_aud_pll_numerator;
	int32_t pllMfnAdjustVal;
	error_t err;
	LWIP_UNUSED_ARG(pvParameters);


	/* Enable GPT Output Compare1 interrupt */
	GPT_EnableInterrupts(GPT3, kGPT_OutputCompare1InterruptEnable);
	/* Enable at the Interrupt and start timer */
	EnableIRQ(GPT3_IRQn);
	GPT_StartTimer(GPT3);


	AppConfigureTimerForUDPSync();
	conn = netconn_new(NETCONN_UDP);
	LWIP_ERROR("Sync task: invalid conn", (conn != NULL), return;);
	netconn_bind(conn, IP_ADDR_ANY, 10);
	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
	PRINTF("Sync task: awaiting first sync message on %p\r\n", conn);
	xSemaphoreGive( xPrintMutex );

	while (1) {
		err = netconn_recv(conn, &net_buf);
		if (err == NO_ERROR) {

			// capture the # of samples collected and reset the counter right away
			uint32_t frozen_samples_collected = samples_collected;
			samples_collected = 0;
			if(netbuf_copy(net_buf, buffer, sizeof(buffer)) != net_buf->p->tot_len) {
				LWIP_DEBUGF(LWIP_DBG_ON, ("netbuf_copy failed\r\n"));
			} else {
				prevUdpMessageNum = udpMessageNum;
				memcpy(&udpMessageNum, &buffer[0], sizeof(udpMessageNum));
				// if UDP sequence number is expected then process the message
				if (((udpMessageNum - prevUdpMessageNum) == 1)
						|| (udpMessageNum == 1)
						|| (udpMessageNum == 0 && prevUdpMessageNum == UINT32_MAX)){
					udp_sync_us = 0; // reset the sync timer
					memcpy(&timerCount, &buffer[4], sizeof(timerCount)); // timer count is the time since the head end sent the last UDP msg in microseconds
					memcpy(&pllMfnAdjustVal, &buffer[8], sizeof(pllMfnAdjustVal));
					memcpy(&udp_master_time_us, &buffer[12], sizeof(udp_master_time_us));
					// manual adjust
					if(pllMfnAdjustVal && pllMfnAdjustVal != 0xFEFEFEFE){
						if (pllMfnAdjustVal < 0){
							xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
							PRINTF("Sync task: got a manual-adjust sync message with timer value of %u, a PLL MFN adjust value of -%i, and a time value of %u\r\n", timerCount, pllMfnAdjustVal, udp_master_time_us);
							xSemaphoreGive( xPrintMutex );
						}
						else{
							xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
							PRINTF("Sync task: got a manual-adjust sync message with timer value of %u, a PLL MFN adjust value of %i, and a time value of %u\r\n", timerCount, pllMfnAdjustVal, udp_master_time_us);
							xSemaphoreGive( xPrintMutex );
						}
						adjust_audio_pll_mfn(pllMfnAdjustVal);

					}
					// auto adjust if we get FFs for the pllMfnAdjustVal
					else if(pllMfnAdjustVal == 0xFEFEFEFE){

						// don't want to tune sample rate to large timer counts
						if (timerCount > MAX_TUNE_TIME_US){
							xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
							PRINTF("Sync task: got an auto-adjust sync message with an out-of-range timer value of %u. The maximum is %u\r\n", timerCount, MAX_TUNE_TIME_US);
							xSemaphoreGive( xPrintMutex );
						}
						else if (timerCount < MIN_TUNE_TIME_US){
							xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
							PRINTF("Sync task: got an auto-adjust sync message with an out-of-range timer value of %u. The minimum is %u\r\n", timerCount, MIN_TUNE_TIME_US);
							xSemaphoreGive( xPrintMutex );
						}

						else{
							xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
							PRINTF("Sync task: got an auto-adjust sync message with a timer value of %u and a time value of %u\r\n", timerCount, udp_master_time_us);
							xSemaphoreGive( xPrintMutex );
							tune_sample_rate(timerCount, frozen_samples_collected, AUDIO_SAMPLE_RATE);
						}
					}
					// no adjust if we get 0s for the pllMfnAdjustVal
					else{
						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
						PRINTF("Sync task: got a no-adjust sync message with timer value of %u and a time value of %u\r\n", timerCount, udp_master_time_us);
						xSemaphoreGive( xPrintMutex );

					}
					memcpy(&node_num_byte, &buffer[16], sizeof(node_num_byte));
					if(node_num_byte & 0x80){ // check if a pwm duty cycle change is requested via the MSB in the node num byte
						intended_node_num = node_num_byte & 0x7F; // mask out the controlling bit
						if(intended_node_num == node_num || intended_node_num == 0x7F){ // 0x7F here means all nodes are requested to change
							uint8_t pwm_payload;
//							uint8_t requested_duty_cycle = 0;
							memcpy(&pwm_payload, &buffer[17], sizeof(pwm_payload));
//							if (pwm_payload > 99){
//								requested_duty_cycle = 99;
//							}
//							else requested_duty_cycle = pwm_payload;

// T.S. lighting style has changed.							SHIELD_UpdatePwmDutyCycle(requested_duty_cycle);
						}
					}
				}
				else {
					xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
					PRINTF("Sync task: got unexpected sync message. got message number %u with a difference of %u\r\n", udpMessageNum, (udpMessageNum - prevUdpMessageNum));
					xSemaphoreGive( xPrintMutex );

				}
			}
		}
		netbuf_delete(net_buf);
	}

}
#endif


#if(0)
extern struct STR_AudioQueue audioQueue;
void tcp_audio_task(void *pvParameters)
{
	TickType_t wakeTimer = 0;
	struct netconn *conn, *newconn = NULL;
	uint8_t *audioBufferPtr;
	uint32_t audioSampleIndex;

	error_t err = NO_ERROR;
	//status_t pdm_status;
//	uint32_t ulNotificationValue;
//	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000);//( 200 );
//	static uint8_t audio_sequence_num = 0;


	LWIP_UNUSED_ARG(pvParameters);
	memset(audio_ch_data, 0, sizeof(audio_ch_data));
	/* Create a new connection identifier. */
	/* Bind connection to well known port number 7. */
	xSemaphoreTake( xLoggingMutex, (TickType_t) portMAX_DELAY);
		audioQueue.size = 320000;
		audioQueue.head = 0;
		audioQueue.tail = 0;
	xSemaphoreGive( xLoggingMutex);

	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, IP_ADDR_ANY, 7);
	LWIP_ERROR("Audio task: invalid conn", (conn != NULL), vTaskDelete(NULL););

	/* Tell connection to go into listening mode. */
	netconn_listen(conn);

	/* Store the handle of the calling task. */
	xTaskToNotify = xTaskGetCurrentTaskHandle();
	//PRINTF("Audio task: Audio initialized.\n\r");
	while (1)
	{
		uint32_t sent_audio_frames = 0;
//		ip_addr_t client_address;
//		uint16_t client_port;
//		uint8_t header[HEADER_SIZE];
		/* Grab new connection. */
		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Audio task: awaiting new connection %p\r\n", newconn);
		xSemaphoreGive( xPrintMutex );
#if(0)
		// for debug... so I don't have to wait for a connection.
		err = netconn_accept(conn, &newconn);

		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Audio task: accepted new connection %p\r\n", newconn);
		xSemaphoreGive( xPrintMutex );

		err |= netconn_peer(newconn, &client_address, &client_port);
#endif
		/* Process the new connection. */
		vTaskDelay(500);
		//PDM_Reset(PDM);
	//	PDM_Enable(PDM, true);
	//	vTaskDelay(500);
	//	PDM_TransferReceiveEDMA(PDM, &s_pdmRxHandle_0, &pdmXfer);
		if (err == NO_ERROR) {
			do {

				/* Wait to be notified that the transmission is complete.  Note
				    the first parameter is pdTRUE, which has the effect of clearing
				    the task's notification value back to 0, making the notification
				    value act like a binary (rather than a counting) semaphore.  */
//				ulNotificationValue = ulTaskNotifyTakeIndexed( xArrayIndex,
//						pdTRUE,
//						xMaxBlockTime );
//				if( ulNotificationValue == 1 )
//				{
//					/* The transmission ended as expected. */
//				}
//				else
//				{
//					//PRINTF("Audio task: 200ms timeout occurred on index %u at sent frame %u\n", s_writeIndex, sent_frames);
//				}
// copy to circular buffer.
#if AUDIO_SAMPLE_RATE != 16000
		if youre reading this, the code below here assumes 16kHz sampling and needs to be changed accordingly.
#endif
				EDMA_TcdDisableInterrupts((edma_tcd_t *)&s_pdmRxHandle_0, (uint32_t)kEDMA_MajorInterruptEnable);
				if(pingPong_audioBufferDataPtr != NULL){
					audioBufferPtr = (uint8_t *)pingPong_audioBufferDataPtr;
					pingPong_audioBufferDataPtr = NULL;	// catch errors / overflows by setting this to NULL.
					if(dataFault == 1){
						memset(audioBufferPtr, 0, BUFFER_SIZE_PDM);		// i'd rather data be 0, than be messed up.
						dataFault = 0;
					}
					EDMA_TcdEnableInterrupts((edma_tcd_t *)&s_pdmRxHandle_0, (uint32_t)kEDMA_MajorInterruptEnable);


					int16_t samples16bit[3][16];
					int32_t sample32bit;
					uint32_t index;


					for(audioSampleIndex = 0; audioSampleIndex < BUFFER_SIZE_PDM; audioSampleIndex += (12*16)){
						for(index = 0; index < 16; index++){
							sample32bit = *(int32_t *)(audioBufferPtr);
//							sample32bit = sample32bit / 4096;	// the 8 LSb are always 0.  THen we move another 8 bits.
							sample32bit = sample32bit >> 12;
							if(sample32bit >= 32767)
								sample32bit = 32767;
							else if(sample32bit <= -32767)
								sample32bit = -32767;
							samples16bit[0][index] = sample32bit;

							audioBufferPtr += 4;

							sample32bit = *(int32_t *)(audioBufferPtr);
//							sample32bit = sample32bit / 4096;	// the 8 LSb are always 0.  THen we move another 8 bits.
							sample32bit = sample32bit >> 12;
							if(sample32bit >= 32767)
								sample32bit = 32767;
							else if(sample32bit <= -32767)
								sample32bit = -32767;
							samples16bit[1][index] = sample32bit;

							audioBufferPtr += 4;

							sample32bit = *(int32_t *)(audioBufferPtr);
//							sample32bit = sample32bit / 4096;	// the 8 LSb are always 0.  THen we move another 8 bits.
							sample32bit = sample32bit >> 12;
							if(sample32bit >= 32767)
								sample32bit = 32767;
							else if(sample32bit <= -32767)
								sample32bit = -32767;
							samples16bit[2][index] = sample32bit;

							audioBufferPtr += 4;
						}

						enqueueAudioElement(samples16bit[0], samples16bit[1], samples16bit[2], 16);

					}


				}else{
					EDMA_TcdEnableInterrupts((edma_tcd_t *)&s_pdmRxHandle_0, (uint32_t)kEDMA_MajorInterruptEnable);
				//	vTaskDelay(50);	// we should be interrupting every 100mS based on microphone sampling rate.
					xTaskDelayUntil( &wakeTimer, 	20 / portTICK_PERIOD_MS );
				}
#if(0)

				header[0] = 0xFF;
				header[1] = 0xFE;
				header[2] = 0xFF;
				header[3] = 0xFE;
				header[4] = 0xFF;
				header[5] = 0xFE;
				header[6] = 0xFF;
				header[7] = 0xFE;
				header[8]= 0; // leave channel index as zero because this is interleaved data
				header[9] = audio_sequence_num;

				memcpy(&header[10], &timestamp_us, sizeof(timestamp_us));
				err = netconn_write(newconn, &header, HEADER_SIZE, NETCONN_NOCOPY);
				err |= netconn_write(newconn, (uint8_t *)(audio_ch_data), CHUNK_SIZE*CHUNKS_TO_SEND, NETCONN_COPY);

				audio_sequence_num++;
				if(audio_sequence_num > MAX_SEQ_NUM){
					audio_sequence_num = 0;
				}

				if(err == NO_ERROR)
				{
					sent_audio_frames++;
				}


				if (err != NO_ERROR) {
					xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
					PRINTF("Audio task: netconn_write: error \"%s\"\r\n", lwip_strerr(err));
					xSemaphoreGive( xPrintMutex );

				}
#endif

			} while(err == NO_ERROR);
			//PDM_TransferAbortReceiveEDMA(DEMO_PDM, &s_pdmRxHandle_0);
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Audio task: Got EOF, looping. %u sent\r\n", sent_audio_frames);
			xSemaphoreGive( xPrintMutex );

			/* Close connection and discard connection identifier. */
			netconn_close(newconn);
			netconn_delete(newconn);
		}
		else{
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Audio task: netconn connection error \"%s\"\r\n", lwip_strerr(err));
			xSemaphoreGive( xPrintMutex );
			/* Close connection and discard connection identifier. */
			netconn_close(newconn);
			netconn_delete(newconn);
		}

	}
}
#endif



