/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "lwip/opt.h"


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>

#include "enet_ethernetif.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_phy.h"
#include "fsl_semc.h"
#include "fsl_wdog.h"
#include "fsl_common.h"

#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/ip.h"
#include "lwip/netifapi.h"
#include "lwip/sockets.h"
#include "netif/etharp.h"

#include "httpsrv.h"
#include "lwip/apps/mdns.h"


#include "fsl_phyksz8081.h"

#include "fsl_enet_mdio.h"
#include "fsl_enet.h"
#include "fsl_iomuxc.h"

#include "peripherals.h"

#include "app_shared.h"

#include "app_accel.h"

#include "app_microwave.h"

#include "app_PIR.h"

#include "app_Jack.h"

#include "app_video.h"

#include "app_board.h"

//#include "httpsclient.h"
//#include "ksdk_mbedtls.h"
//#include "mflash_drv.h"
//#include "mcuboot_app_support.h"
//#include "ota_config.h"


#include "app_logging.h"

#include "app_power.h"

#include "app_config.h"
#include "app_alarm.h"
#include "simpleFlashSupport.h"
#include "app_storage.h"
#include "phy_ksz9563.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @TEST_ANCHOR */




#define EXAMPLE_SEMC               SEMC
#define EXAMPLE_SEMC_START_ADDRESS (0x80000000U)
#define EXAMPLE_SEMC_CLK_FREQ      CLOCK_GetRootClockFreq(kCLOCK_Root_Semc)

// MULTISENSOR SETTINGS
#define EXAMPLE_PHY_ADDRESS BOARD_ENET1_PHY_ADDRESS
/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops
/* ENET instance select. */
#define EXAMPLE_NETIF_INIT_FN ethernetif1_init



/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetRootClockFreq(kCLOCK_Root_Bus)


#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

#ifndef HTTPD_DEBUG
#define HTTPD_DEBUG LWIP_DBG_ON
#endif
#ifndef HTTPD_STACKSIZE
#define HTTPD_STACKSIZE DEFAULT_THREAD_STACKSIZE
#endif
#ifndef HTTPD_PRIORITY
#define HTTPD_PRIORITY DEFAULT_THREAD_PRIO
#endif
#ifndef DEBUG_WS
#define DEBUG_WS 0
#endif

#define CGI_DATA_LENGTH_MAX (96)

#define MDNS_HOSTNAME "lwip-http"


/*******************************************************************************
 * Task Priorities - higher number == more priority
 ******************************************************************************/

#define lpi2c_codec_accel_task_PRIORITY (configMAX_PRIORITIES - 2)
#define lpi2c_camera_task_PRIORITY (configMAX_PRIORITIES - 3)

#define tcp_accel_task_PRIORITY	(configMAX_PRIORITIES - 4)
#define tcp_audio_task_PRIORITY	(configMAX_PRIORITIES - 4)
#define tcp_vid_task_PRIORITY	(TCPIP_THREAD_PRIO - 1)
#define udp_sync_task_PRIORITY	(configMAX_PRIORITIES - 1)
#define accel_polling_task_PRIORITY (configMAX_PRIORITIES - 2)
#define shield_task_PRIORITY (configMAX_PRIORITIES - 4)

#define DEFAULT_THREAD_STACK_SIZE	2000



/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
SemaphoreHandle_t xMutex;
SemaphoreHandle_t xPrintMutex;
SemaphoreHandle_t xI2CSemaphore;
EventGroupHandle_t configEventGroup; // used for stopping other tasks when config is in flux

extern SemaphoreHandle_t xJackMutex;
extern SemaphoreHandle_t xJackConnectionMutex;
extern SemaphoreHandle_t xJPEGVideoMutex;
extern SemaphoreHandle_t xVideoTargetMutex;

extern TimerHandle_t xTimerHandle_video;
extern void vTimerCallBack_video( TimerHandle_t xTimer );

uint32_t node_num = 0;
uint32_t last_reset_cause = 0; // follows the description of the reset cause field in the jack protocol's device ID message
uint32_t wdog_networkActivity = 0x5A5A5A5A;
uint64_t runtime_uS = 0;
uint64_t currentTime_uS = 0;
uint32_t makeBootloadHappen = 0;

struct sys_stat_s sys_stats;



static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle   = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

static struct netif netif;

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_InitModuleClock(void)
{
	const clock_sys_pll1_config_t sysPll1Config = {
			.pllDiv2En = true,
	};
	CLOCK_InitSysPll1(&sysPll1Config);

	//#if BOARD_NETWORK_USE_100M_ENET_PORT  TS
	//    clock_root_config_t rootCfg = {.mux = 4, .div = 10}; /* Generate 50M root clock. */
	//    CLOCK_SetRootClock(kCLOCK_Root_Enet1, &rootCfg);
	//#else
	clock_root_config_t rootCfg = {.mux = 4, .div = 4}; /* Generate 125M root clock. */
	CLOCK_SetRootClock(kCLOCK_Root_Enet2, &rootCfg);
	//#endif
}

void IOMUXC_SelectENETClock(void)
{
	//#if BOARD_NETWORK_USE_100M_ENET_PORT
	//    IOMUXC_GPR->GPR4 |= IOMUXC_GPR_GPR4_ENET_REF_CLK_DIR_MASK; /* 50M ENET_REF_CLOCK output to PHY and ENET module. */
	//#else
	IOMUXC_GPR->GPR5 |= IOMUXC_GPR_GPR5_ENET1G_RGMII_EN_MASK; /* bit1:iomuxc_gpr_enet_clk_dir
                                                                 bit0:GPR_ENET_TX_CLK_SEL(internal or OSC) */
	//#endif
}

void BOARD_ENETFlexibleConfigure(enet_config_t *config)
{
	//#if BOARD_NETWORK_USE_100M_ENET_PORT
	//    config->miiMode = kENET_RmiiMode;
	//#else
	config->miiMode = kENET_RgmiiMode;
	//#endif
}

void WDOG1_IRQHandler(void)
{
	WDOG_ClearInterruptStatus(WDOG1, kWDOG_InterruptFlag);
	/* User code. User can do urgent case before timeout reset.
	 * IE. user can backup the ram data or ram log to flash.
	 * the period is set by config.interruptTimeValue, user need to
	 * check the period between interrupt and timeout.
	 */
}
void WDOG_Start(void){

	wdog_config_t wdogConfig;
	WDOG_GetDefaultConfig(&wdogConfig);
	wdogConfig.timeoutValue = 0xFU; /* Timeout value is (0xF + 1)/2 = 8 sec. */
	wdogConfig.enableInterrupt    = false;
	wdogConfig.interruptTimeValue = 0x4U; /* Interrupt occurred (0x4)/2 = 2 sec before WDOG timeout. */
	wdogConfig.workMode.enableDebug = true; // hopefully fixes errors while debugging
	WDOG_Init(WDOG1, &wdogConfig);
//	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
	WDOG_Refresh(WDOG1);
	PRINTF("--- wdog Init done---\r\n");
//	xSemaphoreGive( xPrintMutex );
//	WDOG_Refresh(WDOG1);

}


void getHHMMSS(uint8_t *hh, uint8_t *mm, uint8_t *ss){
	uint64_t tempTime;

	tempTime = currentTime_uS / 1000000;	// for our sanity, just convert to seconds right now.
	tempTime = tempTime % (86400);			// 3600 * 24.
	*hh = tempTime / (3600);    			//(1000000 * 60 * 60);
	tempTime = tempTime - (*hh * (3600));		//(1000000 * 60 * 60);

	*mm = tempTime / (60);
	tempTime = tempTime - (*mm * 60);

	*ss = tempTime;
	*hh = *hh % 24;
}



#if HTTPSRV_CFG_WEBSOCKET_ENABLED
/*
 * Echo plugin code - simple plugin which echoes any message it receives back to
 * client.
 */
uint32_t ws_echo_connect(void *param, WS_USER_CONTEXT_STRUCT context)
{
#if DEBUG_WS
	PRINTF("WebSocket echo client connected.\r\n");
#endif
	return (0);
}

uint32_t ws_echo_disconnect(void *param, WS_USER_CONTEXT_STRUCT context)
{
#if DEBUG_WS
	PRINTF("WebSocket echo client disconnected.\r\n");
#endif
	return (0);
}

uint32_t ws_echo_message(void *param, WS_USER_CONTEXT_STRUCT context)
{
	WS_send(&context); /* Send back what was received.*/
#if DEBUG_WS
	if (context.data.type == WS_DATA_TEXT)
	{
		/* Print received text message to console. */
		context.data.data_ptr[context.data.length] = 0;
		PRINTF("WebSocket message received:\r\n%s\r\n", context.data.data_ptr);
	}
	else
	{
		/* Inform user about binary message. */
		PRINTF("WebSocket binary data with length of %d bytes received.", context.data.length);
	}
#endif

	return (0);
}

uint32_t ws_echo_error(void *param, WS_USER_CONTEXT_STRUCT context)
{
#if DEBUG_WS
	PRINTF("WebSocket error: 0x%X.\r\n", context.error);
#endif
	return (0);
}

WS_PLUGIN_STRUCT ws_tbl[] = {{"/echo", ws_echo_connect, ws_echo_message, ws_echo_error, ws_echo_disconnect, NULL},
		{0, 0, 0, 0, 0, 0}};
#endif /* HTTPSRV_CFG_WEBSOCKET_ENABLED */


/*!
 * @brief Initializes lwIP stack.
 */
static void stack_init(void)
{

	ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
	ethernetif_config_t enet_config = {
			.phyHandle  = &phyHandle,
			.macAddress = {DEFAULT_NW_MACADDR0, DEFAULT_NW_MACADDR1, DEFAULT_NW_MACADDR2, DEFAULT_NW_MACADDR3, DEFAULT_NW_MACADDR4, DEFAULT_NW_MACADDR5}, //configMAC_ADDR,
	};

	mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

	tcpip_init(NULL, NULL);
	if(defaultConfigFlag){
		enet_config.macAddress[5] += node_num;
		IP4_ADDR(&netif_ipaddr, DEFAULT_NW_IPADDR0, DEFAULT_NW_IPADDR1, DEFAULT_NW_IPADDR2, (DEFAULT_NW_IPADDR3));
		IP4_ADDR(&netif_netmask, DEFAULT_NW_SUBNETMASK0, DEFAULT_NW_SUBNETMASK1, DEFAULT_NW_SUBNETMASK2, DEFAULT_NW_SUBNETMASK3);
		IP4_ADDR(&netif_gw, DEFAULT_NW_GATEWAYADDR0, DEFAULT_NW_GATEWAYADDR1, DEFAULT_NW_GATEWAYADDR2, DEFAULT_NW_GATEWAYADDR3);
	}else{
		memcpy(	enet_config.macAddress, configParam.MACAddr, sizeof(enet_config.macAddress));
		IP4_ADDR(&netif_ipaddr, configParam.ipAddress[0], configParam.ipAddress[1], configParam.ipAddress[2], configParam.ipAddress[3]);
		IP4_ADDR(&netif_netmask, configParam.snMask[0], configParam.snMask[1], configParam.snMask[2], configParam.snMask[3]);
		IP4_ADDR(&netif_gw, configParam.gwAddress[0], configParam.gwAddress[1], configParam.gwAddress[2], configParam.gwAddress[3]);
	}


	netifapi_netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN,
			tcpip_input);
	netifapi_netif_set_default(&netif);
	netifapi_netif_set_up(&netif);

	//    LOCK_TCPIP_CORE();
	//    mdns_resp_init();
	//    mdns_resp_add_netif(&netif, MDNS_HOSTNAME);
	//    mdns_resp_add_service(&netif, MDNS_HOSTNAME, "_http", DNSSD_PROTO_TCP, 80, http_srv_txt, NULL);
	//    UNLOCK_TCPIP_CORE();

	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
	LWIP_PLATFORM_DIAG(("\r\n************************************************"));
	LWIP_PLATFORM_DIAG((" Node %u netconfig\r\n", node_num));
	LWIP_PLATFORM_DIAG(("************************************************"));
	LWIP_PLATFORM_DIAG((" IPv4 Address     : %u.%u.%u.%u", ((u8_t *)&netif_ipaddr)[0], ((u8_t *)&netif_ipaddr)[1],
			((u8_t *)&netif_ipaddr)[2], ((u8_t *)&netif_ipaddr)[3]));
	LWIP_PLATFORM_DIAG((" IPv4 Subnet mask : %u.%u.%u.%u", ((u8_t *)&netif_netmask)[0], ((u8_t *)&netif_netmask)[1],
			((u8_t *)&netif_netmask)[2], ((u8_t *)&netif_netmask)[3]));
	LWIP_PLATFORM_DIAG((" IPv4 Gateway     : %u.%u.%u.%u", ((u8_t *)&netif_gw)[0], ((u8_t *)&netif_gw)[1],
			((u8_t *)&netif_gw)[2], ((u8_t *)&netif_gw)[3]));
	LWIP_PLATFORM_DIAG((" mDNS hostname    : %s", MDNS_HOSTNAME));
	LWIP_PLATFORM_DIAG(("************************************************"));
	xSemaphoreGive( xPrintMutex );


}

void stats_task(void *pvParameters){
	while(1){
		vTaskDelay((1000*60*10) / portTICK_PERIOD_MS ); // save stats every ten minutes
		// save stats here
		saveStats(&sys_stats);
	}

}
#include "fsl_tempsensor.h"
float processorTemperature = 0;
float processorTemperatureMin = 0;
float processorTemperatureMax = 0;
extern uint8_t FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START)];
extern void transferBLandReboot(uint8_t *FW_SwapSpace);

#define RESET_BUTTON_DELAY		10	// this is in units of number of 1/2 seconds (i.e. 10 = 5s delay).
void heartbeat_task(void *pvParameters){
	wdog_config_t wdogConfig;
	int i, reg;
	uint32_t ledsOnOff = 0;
	int wdog_networkActivityTimeout = 120;
	int resetSettingButtonTick = RESET_BUTTON_DELAY;
	TickType_t wakeTimer;
    tmpsns_config_t tempSensorConfig;

	wdog_networkActivity = 0x5A5A5A5A;
	wakeTimer = xTaskGetTickCount();
	/*
	 * wdogConfig->enableWdog = true;
	 * wdogConfig->workMode.enableWait = true;
	 * wdogConfig->workMode.enableStop = false;
	 * wdogConfig->workMode.enableDebug = false;
	 * wdogConfig->enableInterrupt = false;
	 * wdogConfig->enablePowerdown = false;
	 * wdogConfig->resetExtension = false;
	 * wdogConfig->timeoutValue = 0xFFU;
	 * wdogConfig->interruptTimeValue = 0x04u;
	 */

    TMPSNS_GetDefaultConfig(&tempSensorConfig);
    tempSensorConfig.measureMode   = kTEMPSENSOR_ContinuousMode;
    tempSensorConfig.frequency     = 0x03U;
    tempSensorConfig.highAlarmTemp = 80;//DEMO_HIGH_ALARM_TEMP;

    TMPSNS_Init(TMPSNS, &tempSensorConfig);
    TMPSNS_StartMeasure(TMPSNS);
 //   EnableIRQ(DEMO_TEMP_LOW_HIGH_IRQn);		... left this here so that I know we can do it later.
    processorTemperatureMin = processorTemperatureMax = processorTemperature = TMPSNS_GetCurrentTemperature(TMPSNS);

	for(i = 0; i< 3; i++){
		vTaskDelay((500) / portTICK_PERIOD_MS);
		LED_ON_1;
		LED_ON_2;
		LED_ON_3;
		LED_ON_4;

			WDOG_Refresh(WDOG1);

		vTaskDelay((500) / portTICK_PERIOD_MS);
		LED_OFF_1;
		LED_OFF_2;
		LED_OFF_3;
		LED_OFF_4;

			WDOG_Refresh(WDOG1);

	}

	do
	{
		xTaskDelayUntil( &wakeTimer, 	500 / portTICK_PERIOD_MS );

		runtime_uS += 500000;	// this is a 64bit counter.  when counting uS, it will overflow well after I retire.  I.e. in 1.8 x 10^13 seconds. approx. 584542 years.
		currentTime_uS += (500000 + currentTimeIncrementationFactor);
		if(((currentTime_uS / 500000) % MAX_NODE_ADDRESS) == configParam.nodeAddress)
		{
			sendTimeSync();
		}

		if(makeBootloadHappen > 0){
			makeBootloadHappen--;
			if(makeBootloadHappen == 0){
				vTaskSuspendAll();	// pause the scheduler, We aren't coming back.
				__disable_irq();
				LED_OFF_ALL;
				transferBLandReboot((uint8_t *)&FW_SwapSpace);
			}
		}


		if(ledsOnOff > 8){
			HEARTBEAT_LED_OFF;
			ledsOnOff = 0;

			refreshDiagnosticFlags();						// this is in a dumb spot.  And yes, I put it here... TODO put this somewhere better.  T.S.
			if(diagnostic_flags != diagnostic_flagsSent){	// polling this is the only way to know what the UCM thinks / knows and makes sure that it's in sync.
				diagnostic_change_flag |= 1;
			}
		    processorTemperature = TMPSNS_GetCurrentTemperature(TMPSNS);
		    if(processorTemperatureMin > processorTemperature){
		    	processorTemperatureMin = processorTemperature;
		    }
		    if(processorTemperatureMax < processorTemperature){
		    	processorTemperatureMax = processorTemperature;
		    }

		    if((configParam.visible_LEDs_Enable == TRUE) && (getPhyLedState() == FALSE)){
		    	// normal state
		    	setPhyLedsOnOff(TRUE);
		    }else if((configParam.visible_LEDs_Enable == FALSE) && (getPhyLedState() == TRUE)){
		    	// visible LED's are off.
		    	setPhyLedsOnOff(FALSE);
		    }






		} else {
			HEARTBEAT_LED_ON;
			ledsOnOff++;
		}

		if(PB_SETTING_RESET_PUSHED){
			if(resetSettingButtonTick > 1){
				resetSettingButtonTick--;
			}else if(resetSettingButtonTick == 1){
				resetSettingButtonTick = 0;
				WAIT_FOR_FLASH_READY;
				eraseSector(FLASH_ADDR__SETTINGS);
				WAIT_FOR_FLASH_READY;
				eraseSector(FLASH_ADDR__BACKUP_SETTINGS);
				WAIT_FOR_FLASH_READY;
				NVIC_SystemReset();

			}
		}else{
			resetSettingButtonTick = RESET_BUTTON_DELAY;
		}


		if((wdog_networkActivity == 3) || (wdog_networkActivityTimeout > 0) || (wdog_networkActivity == 0x5A5A5A5A)){	// reset watchdog when we've NEVER had network comms, or we have NW comms.

			WDOG_Refresh(WDOG1);

			if((wdog_networkActivity == 3) || (wdog_networkActivity == 0x5A5A5A5A)){
				wdog_networkActivityTimeout = 120;
			}
			if(wdog_networkActivity != 0x5A5A5A5A) {
				wdog_networkActivity = 0;
			}
		}

		if(wdog_networkActivityTimeout > 0){
			wdog_networkActivityTimeout--;
		} else {
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Heartbeat task: Network Failure... resetting soon!!\r\n");
			xSemaphoreGive( xPrintMutex );
		}

	}while(1);



}

/*!
 * @brief The main function containing server thread.
 */


static void main_thread(void *arg)
{
	LWIP_UNUSED_ARG(arg);		// avoids compiler warnings.

#if(0)
	if (xTaskCreate(app_config_task, "app_config", 2048, NULL, DEFAULT_THREAD_PRIO, NULL) != pdPASS)
	{
		PRINTF("Config task creation failed!\r\n");
		while (1)
			;
	}

	// waiting for the config is currently here but later on will be moved into the individual sensors so that the config can change mid operation
			xEventGroupWaitBits(configEventGroup,    /* The event group handle. */
				CONFIGEVENT_CONFIGRDY | CONFIGEVENT_NWRKRDY,        /* The bit pattern the event group is waiting for. */
	                                         pdFALSE,         /* OPPOSITE: B0 and B1 will be cleared automatically. */
	                                         pdTRUE,        /* OPPOSITE: Don't wait for both bits, either bit unblock task. */
	                                         portMAX_DELAY); /* Block indefinitely to wait for the condition to be met. */

#else
		// just call config init right now and force a reset to change settings.
	loadFlashSettings();

	loadStats(&sys_stats);

	PRINTF("Node number assigned: %u \r\n", node_num);
	getUID();
#endif

	stack_init();


#if(0)
#if IS_BOOTLOADER_BUILD_CONFIG
	if (xTaskCreate(httpsclient_task, "httpsclient_task", 2048, NULL, DEFAULT_THREAD_PRIO, NULL) != pdPASS)
	{
		PRINTF("OTA task creation failed!\r\n");
		while (1)
			;
	}
#endif
#endif

#if(1)

#if(1)
	if (xTaskCreate(tcp_video_task, "tcp_video", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("Video TCPIP task creation failed!.\r\n");
		while (1)
			;
	}
#endif

#if(1)

	if (xTaskCreate(video_task, "video_task", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("video task creation failed!.\r\n");
		while (1)
			;
	}
#endif
#if(0)
	// Doesn't work.  I don't know why.  Haven't even looked in to it though.  Might be on PC side.  Don't know.  T.S.
	if (xTaskCreate(udp_sync_task, "udp_sync", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1);
	}
#endif
#if(0)
	if (xTaskCreate(tcp_audio_task, "tcp_audio", (DEFAULT_THREAD_STACK_SIZE), NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1);
	}
#endif
#if(0)
	if (xTaskCreate(tcp_accel_task, "tcp_accel", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}
#endif
#if(1)
	if (xTaskCreate(accel_polling_task, "accel_polling", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("Accelerometer polling task creation failed!.\r\n");
		while (1)
			;
	}
#endif
#if(1)
	if (xTaskCreate(microwave_polling_task, "microwave_polling", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("Microwave polling task creation failed!.\r\n");
		while (1)
			;
	}
#endif
#if(0)
	if (xTaskCreate(tcp_microwave_task, "tcp_microwave", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{		// I haven't really tested this task much... it comes across in a format that I don't support yet... so, it is what it is.  T.S.
		PRINTF("Microwave TCPIP task creation failed!.\r\n");
		while (1)
			;
	}
#endif
#if(1)
	if (xTaskCreate(ADC_polling_task, "ADC_polling", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("ADC polling task creation failed!.\r\n");
		while (1)
			;
	}
#endif
#if(0)
	if (xTaskCreate(tcp_ADC_task, "tcp_ADC", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("ADC TCPIP task creation failed!.\r\n");
		while (1)
			;
	}
#endif
#if(1)
	if (xTaskCreate(tcp_Jack_task, "tcp_Jack", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("Jack TCPIP task creation failed!.\r\n");
		while (1)
			;
	}
#endif
#if(1)
//	when you turn this back on/off, you need to adjust wdog_networkActivity in the jack port.  this should be |= 1 when running, if not |= 3
	if (xTaskCreate(tcp_logging_task, "tcp_logging", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("logging TCPIP task creation failed!.\r\n");
		while (1)
			;
	}
	if (xTaskCreate(logging_task, "logging", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("logging task creation failed!.\r\n");
		while (1)
			;
	}


#endif

#if(0)
	if (xTaskCreate(power_task, "power_task", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("logging TCPIP task creation failed!.\r\n");
		while (1)
			;
	}

#endif
#if(1)

	if (xTaskCreate(app_board_task, "board", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("board task creation failed!.\r\n");
		while (1)
			;
	}
#endif
	if (xTaskCreate(alarm_decision_task, "AlarmDecision", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("alarm Decision task creation failed!.\r\n");
		while (1)
			;
	}

#endif
#if(0)

	if (xTaskCreate(app_storage_task, "Storage", DEFAULT_THREAD_STACK_SIZE, NULL, DEFAULT_THREAD_PRIO, NULL) !=
			pdPASS)
	{
		PRINTF("storage task creation failed!.\r\n");
		while (1)
			;
	}
#endif

#if(1)
		if (xTaskCreate(heartbeat_task, "heartbeat", 1024, NULL, DEFAULT_THREAD_PRIO, NULL) != pdPASS)
		{
			PRINTF("Heartbeat task creation failed!\r\n");
			while (1)
				;
		}
#endif
#if(0)
		if (xTaskCreate(stats_task, "stats", 1024, NULL, DEFAULT_THREAD_PRIO, NULL) != pdPASS)
		{
			PRINTF("Stat task creation failed!\r\n");
			while (1)
				;
		}
#endif
	vTaskDelete(NULL);
}


void i2c_flush(void){
	// i'm sure there's a perfectly good MCUXpresso I2C Flushing function... but I can't find it.
	// note that you need to run this BEFORE pin-init's, because it leaves the bus pins in GPIO state.
	int i, j;
	gpio_pin_config_t gpioOutput_config = {
			.direction = kGPIO_DigitalOutput,
			.outputLogic = 0U,
			.interruptMode = kGPIO_NoIntmode
	};
	gpio_pin_config_t gpioInput_config = {
			.direction = kGPIO_DigitalInput,
			.outputLogic = 0U,
			.interruptMode = kGPIO_NoIntmode
	};

	GPIO_PinInit(GPIO10, 0U, &gpioInput_config);	//SDA	- LPI2C1 - PHY (Default is SPI on phy, but just in case)
	IOMUXC_SetPinMux(
			IOMUXC_GPIO_AD_33_GPIO10_IO00,         /*  */
	        1U);
	GPIO_PinInit(GPIO9, 31U, &gpioOutput_config);	//SCL	- LPI2C1 - PHY (Default is SPI on phy, but just in case)
	IOMUXC_SetPinMux(
			IOMUXC_GPIO_AD_32_GPIO9_IO31,         /*  */
	        1U);
	GPIO_PinInit(GPIO6, 6U, &gpioInput_config);		//SDA	- LPI2C6 - Accel
	GPIO_PinInit(GPIO6, 7U, &gpioOutput_config);	//SCL	- LPI2C6 - Accel
	GPIO_PinInit(GPIO6, 4U, &gpioInput_config);		//SDA	- LPI2C5 - Camera
	GPIO_PinInit(GPIO6, 5U, &gpioOutput_config);	//SCL	- LPI2C5 - Camera

	for(i = 0; i <16; i++){
		GPIO_PinWrite(GPIO9, (31U), 0U);		// SCL
		GPIO_PinWrite(GPIO6, (07U), 0U);		// SCL
		GPIO_PinWrite(GPIO6, (05U), 0U);		// SCL
		for(j = 0; j < 100000; j++);	// this is just over 1ms
		GPIO_PinWrite(GPIO9, (31U), 1U);		// SCL
		GPIO_PinWrite(GPIO6, (07U), 1U);		// SCL
		GPIO_PinWrite(GPIO6, (05U), 1U);		// SCL
		for(j = 0; j < 100000; j++);
	}

	GPIO_PinInit(GPIO10, 0U, &gpioOutput_config);	//SDA
	GPIO_PinInit(GPIO6, 6U, &gpioOutput_config);	//SDA
	GPIO_PinInit(GPIO6, 4U, &gpioOutput_config);	//SDA
	GPIO_PinWrite(GPIO10, (00U), 1U);		// SDA
	GPIO_PinWrite(GPIO6, (06U), 1U);		// SDA
	GPIO_PinWrite(GPIO6, (04U), 1U);		// SDA

	for(i = 0; i <16; i++){
		GPIO_PinWrite(GPIO9, (31U), 0U);		// SCL
		GPIO_PinWrite(GPIO6, (07U), 0U);		// SCL
		GPIO_PinWrite(GPIO6, (05U), 0U);		// SCL
		for(j = 0; j < 100000; j++);
		GPIO_PinWrite(GPIO9, (31U), 1U);		// SCL
		GPIO_PinWrite(GPIO6, (07U), 1U);		// SCL
		GPIO_PinWrite(GPIO6, (05U), 1U);		// SCL
		for(j = 0; j < 100000; j++);
	}

}

status_t BOARD_InitSEMC(void)
{
	semc_config_t config;
	semc_sdram_config_t sdramconfig;
	uint32_t clockFrq = EXAMPLE_SEMC_CLK_FREQ;

	/* Initializes the MAC configure structure to zero. */
	memset(&config, 0, sizeof(semc_config_t));
	memset(&sdramconfig, 0, sizeof(semc_sdram_config_t));

	/* Initialize SEMC. */
	SEMC_GetDefaultConfig(&config);
	config.dqsMode = kSEMC_Loopbackdqspad; /* For more accurate timing. */
	SEMC_Init(SEMC, &config);

	/* Configure SDRAM. */
	sdramconfig.csxPinMux           = kSEMC_MUXCSX0;
	sdramconfig.address             = 0x80000000;
	sdramconfig.memsize_kbytes      = 2 * 32 * 1024;       /* 64MB = 2*32*1024*1KBytes*/
	sdramconfig.portSize            = kSEMC_PortSize32Bit; /*two 16-bit SDRAMs make up 32-bit portsize*/
	sdramconfig.burstLen            = kSEMC_Sdram_BurstLen8;
	sdramconfig.columnAddrBitNum    = kSEMC_SdramColunm_9bit;
	sdramconfig.casLatency          = kSEMC_LatencyThree;
	sdramconfig.tPrecharge2Act_Ns   = 15; /* tRP 15ns */
	sdramconfig.tAct2ReadWrite_Ns   = 15; /* tRCD 15ns */
	sdramconfig.tRefreshRecovery_Ns = 70; /* Use the maximum of the (Trfc , Txsr). */
	sdramconfig.tWriteRecovery_Ns   = 2;  /* tWR 2ns */
	sdramconfig.tCkeOff_Ns =
			42; /* The minimum cycle of SDRAM CLK off state. CKE is off in self refresh at a minimum period tRAS.*/
	sdramconfig.tAct2Prechage_Ns       = 40; /* tRAS 40ns */
	sdramconfig.tSelfRefRecovery_Ns    = 70;
	sdramconfig.tRefresh2Refresh_Ns    = 60;
	sdramconfig.tAct2Act_Ns            = 2; /* tRC/tRDD 2ns */
	sdramconfig.tPrescalePeriod_Ns     = 160 * (1000000000 / clockFrq);
	sdramconfig.refreshPeriod_nsPerRow = 64 * 1000000 / 8192; /* 64ms/8192 */
	sdramconfig.refreshUrgThreshold    = sdramconfig.refreshPeriod_nsPerRow;
	sdramconfig.refreshBurstLen        = 1;
	sdramconfig.delayChain             = 6; /* For all tempeatures. */

	return SEMC_ConfigureSDRAM(SEMC, kSEMC_SDRAM_CS0, &sdramconfig, clockFrq);
}

void *pvPortCalloc(size_t num, size_t size)
{
	void *ptr;
	int allocSize = num * size;

	ptr = pvPortMalloc(allocSize);
	if (ptr != NULL)
	{
		memset(ptr, 0, allocSize);
	}

	return ptr;
}
/*!
 * @brief Main function.
 */
//extern const clock_audio_pll_config_t audioPllConfig;
void SwTimerCallback_PIR(TimerHandle_t xTimer);
extern void ResetISR(void);
void BOARD_InitPins_REV2(void);

#define NET_ORYX

int main(void)
{
//	uint32_t state;

//	int ret;
	uint16_t resetFlag = 0U;

	BOARD_ConfigMPU();

#if 1 //ORYX
	i2c_flush();

	BOARD_InitPins_REV2();

	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_InitBootPeripherals();

	extern void netUserInit(void);
	netUserInit();
#else
	WDOG_Start();

	i2c_flush();

	BOARD_InitPins_REV2();

	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_InitModuleClock();

	IOMUXC_SelectENETClock();

	BOARD_InitBootPeripherals();

	BOARD_InitEnetPins();			// one of these 2 lines may not be necessary.
	BOARD_InitEnet1GPins();			// one of these 2 lines may not be necessary.
	BOARD_InitMicPins();

	EnableIRQ(ENET_1G_MAC0_Tx_Rx_1_IRQn);
	EnableIRQ(ENET_1G_MAC0_Tx_Rx_2_IRQn);

	mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

	PRINTF("\r\n******** System Start ********\r\n");
	PRINTF("System reset by:");

	resetFlag = WDOG_GetStatusFlags(WDOG1);

	switch (resetFlag & (kWDOG_PowerOnResetFlag | kWDOG_TimeoutResetFlag | kWDOG_SoftwareResetFlag))
	{
	case kWDOG_PowerOnResetFlag:
		PRINTF(" Power On Reset!\r\n");
		last_reset_cause = 0x0; // put into format device ID message expects
		break;
	case kWDOG_TimeoutResetFlag: // watchdog reset
		PRINTF(" Time Out Reset!\r\n");
		last_reset_cause = 0x1;
		break;
	case kWDOG_SoftwareResetFlag:
		PRINTF(" Software Reset!\r\n");
		last_reset_cause = 0x2;
		break;
	default:
		PRINTF(" Error status!\r\n");
		last_reset_cause = 0x4;
		break;
	}

	// Semaphores.
	xMutex = xSemaphoreCreateMutex();
	xPrintMutex = xSemaphoreCreateMutex();
	xAccelMutex = xSemaphoreCreateMutex();
	xMicrowaveMutex = xSemaphoreCreateMutex();
	xADCMutex = xSemaphoreCreateMutex();
	xJackMutex = xSemaphoreCreateMutex();
	xJackConnectionMutex = xSemaphoreCreateMutex();
	xLoggingMutex = xSemaphoreCreateMutex();
	xJPEGVideoMutex = xSemaphoreCreateMutex();
	xVideoTargetMutex = xSemaphoreCreateMutex();

	xI2CSemaphore = xSemaphoreCreateMutex();

	configEventGroup = xEventGroupCreate();

	// Timers
	// it boggles my mind why "create" and "Start" need to be done before the scheduler.
	// but It doesn't seem to work if we run these two functions after the scheduler has started.
	// which is just dumb.  So, they're here.  Sigh.  T.S.

	xTimerHandle_video = xTimerCreate("videoTimer", pdMS_TO_TICKS( 100 ), pdTRUE, ( void * ) 0, vTimerCallBack_video);
	xTimerStart(xTimerHandle_video, 0);

    PIT_Configuration();
	changePIT_Period_uS(SAMPLING_PERIOD_STARTUP_uS);

    /* THis is the timer that schedules our ADC conversions.  When this overflows, a signal propagates through 	*/
    /* the XBAR switch.  This causes an ADC conversion cycle to happen. Each conversion cycle, is 5 channels 	*/
    /* worth of data collection (4 bytes each).  After 10 pieces of data (40 bytes) we reach the high water 	*/
    /* mark for the ADC FIFO (NO setup required for this one).  The ADC has a direct connection to the DMA 		*/
    /* through the DE register.  The DMA is setup to look at the ADC DE register (triggered on high water mark	*/
    /* of the ADC).  DMA pulls the full 40 bytes that are in the FIFO and causes an interrupt.  This sets the	*/
    /*	g_TransferCounter flag.  The while loop in ADC task looks for this flag and then shuffles the data in to 		*/
    /* its appropriate place.	*/
	PIT_StartTimer(PIT_BASEADDR, PIT_CHANNEL);

    /* create server thread in RTOS */
    if (sys_thread_new("main", main_thread, NULL, HTTPD_STACKSIZE, HTTPD_PRIORITY) == NULL)
        LWIP_ASSERT("main(): Task creation failed.", 0);
#endif

    /* run RTOS */
    vTaskStartScheduler();

//	ota_abort:
    /* should not reach this statement */
    for (;;)
        ;
}


