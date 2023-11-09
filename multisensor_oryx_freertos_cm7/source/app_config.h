/*
 * app_config.h
 *
 *  Created on: Apr. 20, 2023
 *      Author: stippett
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include "math.h"
#include "app_video.h"

/*******************************************************************************
 * Default Config Definitions
 ******************************************************************************/
#define DEFAULT_PIR_THRESHOLD				20					// 0 to 255.
#define DEFAULT_PIR_BLINDTIME				(0.5 * 1000)		// time between detection windows (0.5s to 8s, in 0.5s intervals).
#define DEFAULT_PIR_PULSECOUNT				2					// required number of pulses exceeding thresh. (1 to 4)
#define DEFAULT_PIR_WINDOWTIME				2					// window size (2s to 8s, in 2s intervals)
#define DEFAULT_PIR_HPFCUTOFF				0					// 0 = 0.4Hz, 1 = 0.2Hz
#define DEFAULT_PIR_COUNTMODE				0					// Count with (0) or without (1) BPF sign change.

#define DEFAULT_RADAR_MINRANGE				0.1					// 0 to 50 in 0.1m increments
#define DEFAULT_RADAR_MAXRANGE				10					// 0 to 50 in 0.1m increments
#define DEFAULT_RADAR_MINVELOCITY			(-7.8)				// -7.8m/s to +7.8m/s.  0.1m/s increments.
#define DEFAULT_RADAR_MAXVELOCITY			7.8					// -7.8m/s to +7.8m/s.  0.1m/s increments.
#define DEFAULT_RADAR_MINSIGNAL				0					// 0 dB (min) to 100dB (max) in 0.1dB increments.
#define DEFAULT_RADAR_MAXSIGNAL				100					// 0 dB (min) to 100dB (max) in 0.1dB increments.
#define DEFAULT_RADAR_FREQCHANNEL			1					// 1 to 8.
#define DEFAULT_RADAR_FALSEALARMSURPRESSION	1					// 0 = disabled, 1 = enabled
#define DEFAULT_RADAR_DISTANCETHRESHOLD		3					// 0m to 20m
#define DEFAULT_RADAR_MULTITARGETTRACKINGSPACING 			3					//
#define DEFAULT_RADAR_TIMECONSTANT 			50					//


#define DEFAULT_ACCEL_HPFSETTING			3					// 0 = 0.1Hz, 1=0.5Hz, 2=1Hz, 3=2Hz, 4=4Hz, 5=8Hz, 6=16Hz, 7=32Hz.
#define DEFAULT_ACCEL_LPFSETTING			0					// 0 = 400Hz, 1=80Hz, 2=40Hz, 3=25Hz, 4=12Hz, 5=8Hz, 6=4Hz, 7=Invalid
#define DEFAULT_ACCEL_EVENTWINDOW			1					// 0.5 to 10 (in 0.5s increments)
#define DEFAULT_ACCEL_EVENTCOUNT			3					// 1 to 10
#define DEFAULT_ACCEL_ALARMWINDOW			30					// 1 to 99 (in 1s increments)
#define DEFAULT_ACCEL_SUPERVISIONTOLERANCE	1					// 0=1x, 1=2x, 3=4x, 4=8x.

#define DEFAULT_CAMERA_IPDECORATION			1					// 1 = Add IP Address to camera Image.
#define DEFAULT_CAMERA_MACDECORATION		0					// 1 = Add MAC Address to camera Image.
#define DEFAULT_CAMERA_TIMEDECORATION		1					// 1 = Add Time of day to camera Image.
#define DEFAULT_CAMERA_NODEADDRDECORATION 	0					// 1 = Add Node Address to camera Image.
#define DEFAULT_CAMERA_MOTIONDETECTION		0					// 1= enable motion detection.
#define DEFAULT_CAMERA_EXPOSURECTRL			0
#define DEFAULT_CAMERA_IMAGESELECTION		0						// TBD for now.

#define DEFAULT_HFMEMS_HPFSETTING			0					// TBD
#define DEFAULT_HFMEMS_LPFSETTING			100					// TBD
#define DEFAULT_HFMEMS_THRESHOLD			100					// TBD
#define DEFAULT_HFMEMS_LRINTERIOR			0					// TBD
#define DEFAULT_HFMEMS_PRIVACY				0					// TBD

#define DEFAULT_NODE_NAME					0					// Null terminated ASCII string.  User available space to identify unit.
#define DEFAULT_NODE_ADDRESS				0					// Node Address
#define DEFAULT_GEO_LATITUDE				0					// Format is TBD
#define DEFAULT_GEO_LONGITUDE				0					// Format is TBD
#define DEFAULT_MOUNTROLL					(0)					// read only by UCM / NM.
#define DEFAULT_MOUNTPITCH					(-30)				// read only by UCM / NM.
#define DEFAULT_MOUNTANGLE_Z				0					// read only by UCM / NM.
#define DEFAULT_MOUNTANGLE_RESET			0					// set to 0x00000001 sent here to reset mounting angle to current orientation.
#define DEFAULT_NW_IPADDR0					192					// default IP address, byte 0
#define DEFAULT_NW_IPADDR1					168					// default IP address, byte 1
#define DEFAULT_NW_IPADDR2					0					// default IP address, byte 2
#define DEFAULT_NW_IPADDR3					170					// default IP address, byte 3
#define DEFAULT_NW_SUBNETMASK0				255					// default subnet mask, byte 0
#define DEFAULT_NW_SUBNETMASK1				255					// default subnet mask, byte 1
#define DEFAULT_NW_SUBNETMASK2				255					// default subnet mask, byte 2
#define DEFAULT_NW_SUBNETMASK3				0					// default subnet mask, byte 3
#define DEFAULT_NW_GATEWAYADDR0				192					// default gateway address, byte 0
#define DEFAULT_NW_GATEWAYADDR1				168					// default gateway address, byte 1
#define DEFAULT_NW_GATEWAYADDR2				0					// default gateway address, byte 2
#define DEFAULT_NW_GATEWAYADDR3				1					// default gateway address, byte 3
#define DEFAULT_NW_SETTINGS_BITFLD			0x02				// bit 0: 1=enable SSL, bit 1: 1=enable RSTP, bit 2-31: unused
#define DEFAULT_NW_MACADDR0					0xAA				// default MAC address, byte 0. Only bytes 0-5 are used
#define DEFAULT_NW_MACADDR1					0xBB				// default MAC address, byte 1. Only bytes 0-5 are used
#define DEFAULT_NW_MACADDR2					0xCC				// default MAC address, byte 2. Only bytes 0-5 are used
#define DEFAULT_NW_MACADDR3					0xDD				// default MAC address, byte 3. Only bytes 0-5 are used
#define DEFAULT_NW_MACADDR4					0xEE				// default MAC address, byte 4. Only bytes 0-5 are used
#define DEFAULT_NW_MACADDR5					0xFF				// default MAC address, byte 5. Only bytes 0-5 are used
#define DEFAULT_NW_MACADDR6					0x00				// default MAC address, byte 6. Only bytes 0-5 are used
#define DEFAULT_NW_MACADDR7					0x00				// default MAC address, byte 7. Only bytes 0-5 are used
#define DEFAULT_LED_IRENABLE				1					// 1 = IR LEDs enabled
#define DEFAULT_LED_VISBILEENABLE			1					// 1 = Front Face LED's enabled

#define DEFAULT_WARNING_BITFLD				0x1F				// Bit0:1=LossOfNWLink,Bit1:1=LossOf1588Sync,Bit2: HWFault,Bit3:Tamper,Bit4:inputPowerFault,Bit5:restart,Bit6-31: unused.
#define DEFAULT_ALARM_BITFLD				0x2F				// Bit0:Accel,Bit1:Radar,Bit2:PIR0,Bit3:PIR1,Bit4:Camera,Bit5:HFMEMS,Bit6-31:unused.

#define DEFAULT_PRIVACY_CAMERA				0x00				// 1= camera privacy enabled.  Images are NOT displayed in UCM.

#define DEFAULT_ACCESSLOG_CLEARVAL			0					// defaults to this value after changing and clearing access log


#define MAX_NODE_ADDRESS					50







/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
#define NW_SSL_ENABLED 0 // TODO



/*******************************************************************************
 * Definitions
 ******************************************************************************/
#pragma pack(1)
struct STR_PIRConfig{
	uint32_t	threshold;		// 0 to 255.
	uint32_t	blindTime_mS;	// time between detection windows (0.5s to 8s, in 0.5s intervals).
	uint32_t	pulseCount;		// required number of pulses exceeding thresh. (1 to 4)
	uint32_t	windowTime_S;	// window size (2s to 8s, in 2s intervals)
	uint32_t	HPFCutOff;		// 0 = 0.4Hz, 1 = 0.2Hz
	uint32_t	countMode;		// Count with (0) or without (1) BPF sign change.
};


#pragma pack(1)
struct STR_radarConfig{
	float_t		minRange_m;				// 0 to 50 in 0.1m increments
	float_t		maxRange_m;				// 0 to 50 in 0.1m increments
	float_t		minVelocity_mps;		// -7.8m/s to +7.8m/s.  0.1m/s increments.
	float_t		maxVelocity_mps;		// -7.8m/s to +7.8m/s.  0.1m/s increments.
	float_t		minSignal_dB;			// 0 dB (min) to 100dB (max) in 0.1dB increments.
	float_t		maxSignal_dB;			// 0 dB (min) to 100dB (max) in 0.1dB increments.
	uint32_t	frequencyChannel;		// 1 to 8.
	uint32_t	falseAlarmSuppression;	// 0 = disabled, 1 = enabled.
	float_t		radarDistanceThreshold;	// 0 to 10m.  Closer than this is prox warning for the radar.
	float_t		multiTargetTrackingSpaceing;	//
	float_t		timeConstant;					//

};


#pragma pack(1)
struct STR_accelConfig	{
	uint32_t	HPFSetting;					// 0 = 0.1Hz, 1=0.5Hz, 2=1Hz, 3=2Hz, 4=4Hz, 5=8Hz, 6=16Hz, 7=32Hz.
	uint32_t	LPFSetting;					// 0 = 400Hz, 1=80Hz, 2=40Hz, 3=25Hz, 4=12Hz, 5=8Hz, 6=4Hz, 7=Invalid.
	float_t		eventWindow_s;				// 0.5 to 10 (in 0.5s increments)
	uint32_t	eventCount;					// 1 to 10
	float_t		alarmWindow_s;				// 1 to 99 (in 1s increments)
	uint32_t	supervisionTolerance;		// 0=1x, 1=2x, 2, 3=4x, 4=8x.
};


#pragma pack(1)
struct STR_cameraConfig	{
	uint8_t		decorationIPAddress;	// 1 = Add IP Address to camera Image.
	uint8_t		decorationMACAddress;	// 1 = Add MAC Address to camera Image.
	uint8_t		decorationTimeOfDay;	// 1 = Add Time of day to camera Image.
	uint8_t		decorationNodeAddress;	// 1 = Add Node Address to camera Image.
	uint8_t		decorationTrackingBoxes;	// 1 = Add tracking boxes to the camera image.
	uint8_t		decorationTracking;			// 1 = add tracking to camera image.
	uint8_t		cameraImageSelection;
	uint8_t		enableMotionDetection;	// 1= enable motion detection.
	uint32_t	exposureControl;		// TBD for now.
};

#pragma pack(1)
struct STR_HFMEMSConfig	{
	uint32_t	HPFSetting;
	uint32_t	LPFSetting;
	uint32_t	threshold;
	uint32_t	LRInterior;
	uint32_t 	privacy;
};


#pragma pack(1)
typedef struct STR_ConfigParam {
	// 64 byte password.  Straight ASCII for now.
		// All 64 Bytes must match a stored password (every single byte...
		// even if PW is 2 bytes long).  See PW matrix document.
	// userPW is sent to UCM as 0x00.
	char userPW[64];
	uint64_t serialNumber;
	// all items below are sent to UCM.  PW match above in order to alter the settings.

	// PW L2 items:
	char sensorName[256];	// Null terminated ASCII string.  User available space to identify unit.
	uint32_t nodeAddress;	// Node address.

	// Mounting Location
	double_t GEO_latitude;		// Format is TBD.
	double_t GEO_longitude;		// Format is TBD.

	// Mounting Angle at time of installation.  units are TBD.
//	uint32_t	mountingAngle_x;	//	These are read only by UCM / NM.
//	uint32_t	mountingAngle_y;	//
//	uint32_t	mountingAngle_z;	//
//	uint32_t	resetMountingAngle;	// set to 0x00000001 sent here to reset mounting angle to current orientation.
									// This will always read as 0x00000000.

	float		mountingRoll;
	float		mountingPitch;
	uint32_t	unused_reserved0;
	uint32_t	unused_reserved1;


	// NW settings
	uint8_t		ipAddress[4];		//
	uint8_t		snMask[4];		//
	uint8_t		gwAddress[4];		//
	uint32_t		IPBitField;		// bit 0: 1=enable SSL, bit 1: 1=enable RSTP, bit 2-31: unused
	uint8_t		MACAddr[8];	// Read only for UCM.  Only bytes 0-5 are used.
	float_t 	sideA_DataSharing;
	uint32_t 	sideA_DataSharingSetting;
	float_t 	sideB_DataSharing;
	uint32_t 	sideB_DataSharingSetting;

	// LEDs
	uint32_t	ir_LEDs_Enable;		// 1 = IR LED's enabled
	uint32_t	visible_LEDs_Enable;	// 1 = Front Face LED's enabled

	// sensor settings
	struct STR_accelConfig	accelConfig;	//
	struct STR_radarConfig	radarConfig;	//
	struct STR_PIRConfig	PIR0_Config;	//
	struct STR_PIRConfig	PIR1_Config;	//
	struct STR_cameraConfig	cameraConfig;	//
	struct STR_HFMEMSConfig	HFMEMSConfig;	// HF MEMS

	uint32_t	warningBitfield;		// bitfield that identifies what will raise a warning:
			// Bit0: 1=LossOfNWLink.
			// Bit1: 1=LossOf1588Sync.
			// Bit2: HWFault.
			// Bit3: Tamper.
			// Bit4: inputPowerFault.  Bit5: restart.  Bit6-31: unused.

	uint32_t		alarmBitfield;		// bitfield that identifies what sensors are used to generate 						// an alarm (used together... i.e. not a Logic OR)
			// Bit0: Accel.
			// Bit1: Radar.
			// Bit2: PIR0.
			// Bit3:PIR1.
			// Bit4: Camera.
			// Bit5: HFMEMS.


	uint32_t		neighbourControl;	// bitfield for power control on neighbours.
			// bit 0: Left
			// bit 1: right.

	// PW L3 items:
	uint32_t	cameraPrivacyEnabled;	// 1= camera privacy enabled.  Images are NOT displayed in UCM.

	// PW L4 items:


	// PW L5 items:
	uint32_t	clearAccessLog;		// 1 = clear access log.

	// PW L6 items:
	float 	reserved0f;
	float 	reserved1f;
	float 	reserved2f;
	float 	reserved3f;
	float 	reserved4f;
	float 	reserved5f;
	float 	reserved6f;
	float 	reserved7f;

	int32_t reserved0i;
	int32_t reserved1i;
	int32_t reserved2i;
	int32_t reserved3i;
	int32_t reserved4i;
	int32_t reserved5i;
	int32_t reserved6i;
	int32_t reserved7i;

	uint32_t unusedArray[64];
#if(1)
	// TODO:
	// although these masks are in, they are not saved to flash.  This is a flash saving issue.  If you put these in, it doesn't save to flash before the board crashes.
	// i don't know what the exact root cause is.  To get around this issue, there's a handfull of places where i've added "- (((30 * 40)/8) * 3 * 4)" when the size of
	// the config param needs to be known.  I think there's one or 2 in the load flash function too.  you'll need to remove those to debug this issue.

	uint32_t motionDetectMask[((30 * 40)/8)];		//  1 = use this pixel for motion detect - 1 bit per pixel.  Mask is of resolution 30 x 40.
	uint32_t imageDisplayMask[((30 * 40)/8)];		//  1 = use this pixel for displaying, black out otherwise.  1 bit per pixel.  Mask is of resolution 30 x 40.
	uint32_t proxWarningMask[((30 * 40)/8)];		//  1 = use this pixel for proxWarning.  Not implemented yet.  Mask is of resolution 30 x 40.
#endif


} config_param_t;



struct STR_UCM_PlotMsg{
	// jack msg UCM Plot data for sensors.

	uint32_t accelMagnitude;	//
	uint32_t leftPIRMagnitude;	//
	uint32_t rightPIRMagnitude;	//
	uint32_t HFMEMSMagnitude;	//
	uint32_t cameraMagnitude;	//
	uint32_t radarMagnitude;	//
	float radarXY[5][2];		//

	uint32_t	jpegSize;
	uint8_t		jpegData[JPG_IMAGE_BUFFER_SIZE];
};






/*******************************************************************************
 * Shared Variables
 ******************************************************************************/
extern config_param_t configParam;
extern int defaultConfigFlag;

/*******************************************************************************
 * Functions
 ******************************************************************************/
void app_set_config_param(config_param_t *newConfig);
uint8_t app_parse_config_param(uint8_t* configBytes, config_param_t *parsedConfig);

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void app_config_task(void *pvParameters);
void config_init(void);
uint32_t sanitizeConfig(config_param_t *newCfgPtr, config_param_t *fallBackCfgPtr, uint32_t securityLevel);

#endif /* APP_CONFIG_H_ */
