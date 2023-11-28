/*
 * app_shared.h
 *
 *  Created on: Jun. 21, 2021
 *      Author: stippett
 */

#ifndef APP_SHARED_H_
#define APP_SHARED_H_

//#include "lwip/opt.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <math.h>
#include "FreeRTOS.h"

#include "core/net.h"
#include "debug.h"
#include "task.h"
#include "event_groups.h"
#include "timers.h"
#include "fsl_gpio.h"
#include "node_identity.h"
#include "cr_section_macros.h"
#include "app_audio.h"
#include "fsl_debug_console.h"


//#define I2C_RETRY_TIMES 10		// define this so that our I2C ports don't hang when we pause at a breakpoint.
#include "fsl_lpi2c.h"
#include "MMA8451.h"
#include "senstarCRC.h"


/*******************************************************************************
 * Config Definitions
 ******************************************************************************/
#define IS_BOOTLOADER_BUILD_CONFIG (ConfigName == BL_0x40400)


#define MAX_SEQ_NUM 		200
#define DEVICE_TYPE 		0x00000001
#define FIRMWARE_VERSION	0x00000001 // this should be changed to be synchronized with the image version in mcuboot_app_support.h
#define HARDWARE_VERSION	0x00000001
#if (HARDWARE_VERSION == 0x00000001)
#define SERIAL_NUMBER_BASE	0x1DEF3D10
#endif



/*******************************************************************************
 * Definitions
 ******************************************************************************/
	#define LED_ON_1 	GPIO_PinWrite(GPIO9, (4U), 1U);				// LED
	#define LED_ON_2 	GPIO_PinWrite(GPIO9, (7U), 1U);				// LED
	#define LED_ON_3 	GPIO_PinWrite(GPIO9, (9U), 1U);				// LED
	#define LED_ON_4 	GPIO_PinWrite(GPIO9, (10U), 1U);				// LED
	#define LED_ON_5 	GPIO_PinWrite(GPIO9, (18U), 1U);				// LED
	#define LED_ON_6 	GPIO_PinWrite(GPIO9, (26U), 1U);				// LED
	#define LED_ON_7 	GPIO_PinWrite(GPIO11, (8U), 1U);				// LED
	#define LED_ON_8 	GPIO_PinWrite(GPIO11, (10U), 1U);				// LED

	#define LED_OFF_1 	GPIO_PinWrite(GPIO9, (4U), 0U);				// LED
	#define LED_OFF_2 	GPIO_PinWrite(GPIO9, (7U), 0U);				// LED
	#define LED_OFF_3 	GPIO_PinWrite(GPIO9, (9U), 0U);				// LED
	#define LED_OFF_4 	GPIO_PinWrite(GPIO9, (10U), 0U);				// LED
	#define LED_OFF_5 	GPIO_PinWrite(GPIO9, (18U), 0U);				// LED
	#define LED_OFF_6 	GPIO_PinWrite(GPIO9, (26U), 0U);				// LED
	#define LED_OFF_7 	GPIO_PinWrite(GPIO11, (8U), 0U);				// LED
	#define LED_OFF_8 	GPIO_PinWrite(GPIO11, (10U), 0U);				// LED

	#define LED_ON_ALL	{LED_ON_1; LED_ON_2; LED_ON_3; LED_ON_4; LED_ON_5; LED_ON_6; LED_ON_7; LED_ON_8;}
	#define LED_OFF_ALL	{LED_OFF_1; LED_OFF_2; LED_OFF_3; LED_OFF_4; LED_OFF_5; LED_OFF_6; LED_OFF_7; LED_OFF_8;}

	#define LED_ARRAY_CHARGE_ON  GPIO_PinWrite(GPIO5, (13U), 1U);
	#define LED_ARRAY_CHARGE_OFF GPIO_PinWrite(GPIO5, (13U), 0U);

	#define LED_ARRAY_STROBE_ON		GPIO_PinWrite(GPIO9, (22U), 1U);				// LED
	#define LED_ARRAY_STROBE_OFF 	GPIO_PinWrite(GPIO9, (22U), 0U);				// LED


	#define DIAG_E8_ON		GPIO_PinWrite(GPIO11, (1U), 1U);				// Diagnostic PIN E8
	#define DIAG_E8_OFF		GPIO_PinWrite(GPIO11, (1U), 0U);				// Diagnostic PIN E8

	#define CAMERA_LED_ON			if(configParam.visible_LEDs_Enable == TRUE){LED_ON_1;}else{LED_OFF_1;}
	#define CAMERA_LED_OFF			LED_OFF_1
	#define HEARTBEAT_LED_ON		if(configParam.visible_LEDs_Enable == TRUE){LED_ON_2;}else{LED_OFF_2;}
	#define HEARTBEAT_LED_OFF		LED_OFF_2
	#define PIR_A_LED_ON			if(configParam.visible_LEDs_Enable == TRUE){LED_ON_3;}else{LED_OFF_3;}
	#define PIR_A_LED_OFF			LED_OFF_3
	#define PIR_B_LED_ON			if(configParam.visible_LEDs_Enable == TRUE){LED_ON_4;}else{LED_OFF_4;}
	#define PIR_B_LED_OFF			LED_OFF_4
	#define RADAR_LED_ON			if(configParam.visible_LEDs_Enable == TRUE){LED_ON_5;}else{LED_OFF_5;}
	#define RADAR_LED_OFF			LED_OFF_5
	#define ACCEL_LED_ON			if(configParam.visible_LEDs_Enable == TRUE){LED_ON_6;}else{LED_OFF_6;}
	#define ACCEL_LED_OFF			LED_OFF_6
	#define POWER_FAULT_A_LED_ON	if(configParam.visible_LEDs_Enable == TRUE){LED_ON_7;}else{LED_OFF_7;}
	#define POWER_FAULT_A_LED_OFF	LED_OFF_7
	#define POWER_FAULT_B_LED_ON	if(configParam.visible_LEDs_Enable == TRUE){LED_ON_8;}else{LED_OFF_8;}
	#define POWER_FAULT_B_LED_OFF	LED_OFF_8

#define PIR_LED_OFF					{PIR_A_LED_OFF; PIR_B_LED_OFF;}
#define PIR_LED_ON					{PIR_A_LED_ON; PIR_B_LED_ON;}

	#define EFUSE_A_SHUTDOWN_ASSERT 		GPIO_PinWrite(GPIO9, (6U), 0U);				//
	#define EFUSE_A_SHUTDOWN_UN_ASSERT  	GPIO_PinWrite(GPIO9, (6U), 1U);				//
	#define EFUSE_B_SHUTDOWN_ASSERT  		GPIO_PinWrite(GPIO9, (0U), 0U);				//
	#define EFUSE_B_SHUTDOWN_UN_ASSERT 		GPIO_PinWrite(GPIO9, (0U), 1U);				//

	#define EFUSE_A_FAULT_INPUT				(GPIO_PinRead(GPIO9, (2U)) == 0x01)
	#define EFUSE_B_FAULT_INPUT				(GPIO_PinRead(GPIO9, (16U)) == 0x01)

	#define PHY_PME_FALUT_ASSERTED_INPUT		(GPIO_PinRead(GPIO9, (21U)) == 0x00)		// active low.
	#define PHY_PME_FALUT_UNASSERTED_INPUT		(GPIO_PinRead(GPIO9, (21U)) == 0x01)		// active low.

	#define PHY_RESET_ASSERT					GPIO_PinWrite(GPIO9, (8U), 0U);
	#define PHY_RESET_UNASSERT					GPIO_PinWrite(GPIO9, (8U), 1U);
	#define PHY_INT_ASSERTED					(GPIO_PinRead(GPIO9, (5U)) == 0x01)
	#define PHY_INT_UNASSERTED					(GPIO_PinRead(GPIO9, (5U)) == 0x00)


	#define PB_SETTING_RESET_PUSHED				(GPIO_PinRead(GPIO6, (0U)) == 0x00)

	#define CAMERA_RESET_ASSERT			GPIO_PinWrite(GPIO5, (15U), 0U)				// Camera RESETB - put in to RESET
	#define CAMERA_RESET_UNASSERT		GPIO_PinWrite(GPIO5, (15U), 1U)				// Camera RESETB - put in to RESET


	#define PHY_RESET_UNASSERT					GPIO_PinWrite(GPIO9, (8U), 1U);


// event bits for configEventGroup
#define CONFIGEVENT_CONFIGRDY 	(1 << 0)
#define CONFIGEVENT_NWRKRDY 	(1 << 1)


#define SIZE_OF_LN_DETECT_CFG       12
union LN_detect_cfg{
	uint8_t byte_array[SIZE_OF_LN_DETECT_CFG];
	uint16_t word_array[SIZE_OF_LN_DETECT_CFG /2];
	uint32_t dword_array[SIZE_OF_LN_DETECT_CFG /4];
	struct{

		uint8_t Event_Count;            //used to define a the event count which will trigger an alarm
		uint8_t Event_Window;              //used for "event window" which is an event retrigger timer
		uint8_t Alarm_Window;            //used for "alarm window" which is a flush time for event tracking
		uint8_t Alarm_Hold_Time;			//number of seconds the an alarm is hold in the active state

		uint8_t HPF_val;				//detection filter lower corner LUT val
		uint8_t LPF_val;				//detection filter upper corner LUT val
		uint8_t SupMode;              //Supervision Mode
		uint8_t Event_Threshold;		//event threshold - self explanitory

		uint8_t Lighting_Mode;		//define Light behavior
		uint8_t Brightness_Nominal;	//LED brightness in lighting mode
		uint8_t Brightness_Alarm;		//LED brightness in alarm, if used
		uint8_t Zone_Seg;				//store zone info which may be used in broadcast messages

                uint8_t Brightness_lvl2;              //multibrightness expansion
                uint8_t reserve[3];

	};

};

#define SIZE_OF_LN_RF_CFG       20

union LN_RF_cfg{
	uint8_t byte_array[SIZE_OF_LN_RF_CFG];
	uint16_t word_array[SIZE_OF_LN_RF_CFG /2];
	uint32_t dword_array[SIZE_OF_LN_RF_CFG /4];
	struct{

		uint8_t MAC_address[6];
		uint8_t Coordinator_MAC_address[6];

		uint16_t short_address;
		uint16_t PAN_ID;

		uint8_t channel;
		uint8_t index_num;
		uint16_t fw_version;

		};
	};


struct Proc_config_S{
	union LN_detect_cfg detect;
	union LN_RF_cfg RF;
};

struct detector{

	 uint8_t event_count;
	 uint8_t event_count_threshold;
	 uint8_t alarm;
	 uint8_t send_flag;

	 uint16_t event_timer;
	 uint16_t event_timer_reload;

	 uint16_t alarm_timer;
	 uint16_t alarm_timer_reload;

	 uint16_t alarm_hold_timer_reload;
	 uint16_t res;

	 uint32_t liftime_event_count;
	 uint32_t liftime_alarm_count;
	 uint32_t liftime_event_TO_count;
	 uint32_t uptime;

 };
extern struct Proc_config_S Proc_config;


//////////////////////////////////////
// Structure for storing time stamp //
struct time_data_s
{
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint8_t date;
  uint8_t month;
  uint8_t year;
};
//////////////////////////////////////


////////////////////////////////////////////
// Structure for storing temperature data //
struct tmpr_data_s
{
  uint32_t temperature;
  struct time_data_s time;
};
////////////////////////////////////////////

////////////////////////////////////////////
// Structure for storing power statistics //
struct misc_stat_s
{
  uint32_t boot_count;
  uint32_t hour_a;               // runtime
  uint32_t hour_b;               // runtime
};
////////////////////////////////////////////

////////////////////////////////////////////
// Structure for storing power statistics //
struct power_stat_s
{
  uint32_t fault6V5_count;
  uint32_t fault3V3_count;
  uint32_t faultSideAPower_count;
  uint32_t faultSideBPower_count;
};
////////////////////////////////////////////



////////////////////////////////////////////
// Structure for storing network statistics //
struct network_stat_s
{
  uint32_t faultSideANetwork_count;
  uint32_t faultSideBNetwork_count;
  uint32_t UCM_connection_count;
};
////////////////////////////////////////////



////////////////////////////////////////////
// Structure for storing sensor statistics //
struct sensor_stat_s
{
  uint32_t faultAccel_count;
  uint32_t faultLPIR_count;
  uint32_t faultRPIR_count;
  uint32_t faultRadar_count;
  uint32_t faultHFMEMS_count;
  uint32_t faultCamera_count;
};
////////////////////////////////////////////


////////////////////////////////////////////
// Structure for storing system statistics
struct sys_stat_s
{
  struct tmpr_data_s  	gmin;
  struct tmpr_data_s  	gmax;
  struct power_stat_s	pwrstat;
  struct network_stat_s	nwkstat;
  struct sensor_stat_s 	sensorstat;
  struct misc_stat_s  	mstat;
};
extern struct sys_stat_s sys_stats;
////////////////////////////////////////////




/*******************************************************************************
 * Shared Variables
 ******************************************************************************/
extern 			SemaphoreHandle_t xPrintMutex;
extern 			SemaphoreHandle_t xMutex;
extern 			SemaphoreHandle_t xAccelMutex;
extern 			SemaphoreHandle_t xI2CSemaphore;

extern			EventGroupHandle_t configEventGroup;

extern uint32_t timestamp_us; 				// for capturing udp_sync_us at the time accel data is ready to be sent
extern uint32_t udp_sync_us;
extern uint32_t udp_master_time_us;

extern uint32_t node_num;					// which node number in the system.  Based on the unique ID of the processor.
extern uint32_t last_reset_cause;

extern uint32_t wdog_networkActivity;		// used to flag that there's network activity.  So we can reset the board if network goes down.

extern uint32_t power_flags;
extern uint32_t diagnostic_flags;
extern float processorTemperature, processorTemperatureMin, processorTemperatureMax;
/*******************************************************************************
 * Functions
 ******************************************************************************/
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

void processADCResults(uint16_t* pirResultsLeft, uint16_t* pirResultsRight, uint16_t* powerMonResults, uint8_t resultsExpected);

void app_board_task(void *pvParameters);
/*******************************************************************************
 * Tasks
 ******************************************************************************/

#endif /* APP_SHARED_H_ */
