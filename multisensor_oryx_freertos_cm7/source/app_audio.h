/*
 * app_audio.h
 *
 *  Created on: Apr. 19, 2021
 *      Author: stippett
 */

#ifndef APP_AUDIO_H_
#define APP_AUDIO_H_
#include "stdint.h"
/*******************************************************************************
 * Config Definitions
 ******************************************************************************/
#define NUM_MICS 3
#define MAX_TUNE_TIME_US	15000000 // only tune to the udp sync msg if timerCount is less than 15 seconds
#define MIN_TUNE_TIME_US	1000000 // only tune to the udp sync msg if timerCount is more than 1 second

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PDM_ENABLE_CHANNEL0_RIGHT  (0U) // hoping the left and rights are actually swapped. documentation points to the left channel being the first in line
#define PDM_ENABLE_CHANNEL0_LEFT (1U)
#define PDM_ENABLE_CHANNEL1_RIGHT  (2U)
#define PDM_ENABLE_CHANNEL1_LEFT (3U)
#define PDM_ENABLE_CHANNEL2_RIGHT  (4U)
#define PDM_ENABLE_CHANNEL2_LEFT (5U)

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void tcp_audio_task(void *pvParameters);
void tcp_audio_task_tim(void);//(void *pvParameters);
void udp_sync_task(void *pvParameters);
uint32_t getHFMEMSMagnitude(void);
int32_t	getHFMEMSError(void);
int audio_init(void);
#endif /* APP_AUDIO_H_ */

