/*
 * app_config.c
 *
 *  Created on: Apr. 20, 2023
 *      Author: stippett
 */
#include "app_config.h"
#include "app_shared.h"
#include "app_jack.h"
#include "app_microwave.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_FLASH_ENABLED 0

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void config_init(void);
static void config_reinit(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
int defaultConfigFlag = 1;

config_param_t configParam = {
		.serialNumber = 0x0102030405060708,
		.nodeAddress = 0,
		.GEO_latitude = 0,
		.GEO_longitude = 0,
		.visible_LEDs_Enable = 1,
		.ir_LEDs_Enable = 1,
		.cameraConfig.decorationIPAddress = 1,
		.cameraConfig.decorationTimeOfDay = 1,
		.ipAddress = {192,168,0,170},
		.snMask = {255,255,255,0},
		.gwAddress = {192,168,0,1},
};
/*******************************************************************************
 * Code
 ******************************************************************************/

// outward facing function for handling changes to config param
void app_set_config_param(config_param_t *newConfig){
	// notify config task that changes to config are occurring

	//check new config for special values that require action (like reset mounting angle)
	//
}

// outward facing function for parsing out the config param into a given empty struct from an array of bytes
// return a non-zero value if there is an error in the parsing
uint8_t app_parse_config_param(uint8_t* configBytes, config_param_t *parsedConfig){


	return 0;
}

// local function for changing the config param from the current global one to the given struct. doesn't handle anything else
static inline void swap_config(config_param_t *newConfig){
	//configParam.userPW[64]
	//configParam.serialNumber

	memcpy(&configParam, newConfig, sizeof(configParam));


#if(0)
		// PW L2 items:
		configParam.sensorName[0] = newConfig->sensorName[0]; // TODO
		configParam.nodeAddress = newConfig->nodeAddress;

		// Mounting Location
		configParam.GEO_latitude = newConfig->GEO_latitude;
		configParam.GEO_longitude = newConfig->GEO_longitude;

		// Mounting Angle at time of installation
		configParam.mountingAngle_x = newConfig->mountingAngle_x;
		configParam.mountingAngle_y = newConfig->mountingAngle_y;
		configParam.mountingAngle_z = newConfig->mountingAngle_z;
		configParam.resetMountingAngle = newConfig->resetMountingAngle; // will reset to this value after change

		// NW settings
		configParam.ipAddress[0] = newConfig->ipAddress[0];
		configParam.ipAddress[1] = newConfig->ipAddress[1];
		configParam.ipAddress[2] = newConfig->ipAddress[2];
		configParam.ipAddress[3] = newConfig->ipAddress[3];
		configParam.snMask[0] = newConfig->snMask[0];
		configParam.snMask[1] = newConfig->snMask[1];
		configParam.snMask[2] = newConfig->snMask[2];
		configParam.snMask[3] = newConfig->snMask[3];
		configParam.gwAddress[0] = newConfig->gwAddress[0];
		configParam.gwAddress[1] = newConfig->gwAddress[1];
		configParam.gwAddress[2] = newConfig->gwAddress[2];
		configParam.gwAddress[3] = newConfig->gwAddress[3];
		configParam.IPBitField = newConfig->IPBitField;
		configParam.MACAddr[0] = newConfig->MACAddr[0];
		configParam.MACAddr[1] = newConfig->MACAddr[1];
		configParam.MACAddr[2] = newConfig->MACAddr[2];
		configParam.MACAddr[3] = newConfig->MACAddr[3];
		configParam.MACAddr[4] = newConfig->MACAddr[4];
		configParam.MACAddr[5] = newConfig->MACAddr[5];
		configParam.MACAddr[6] = newConfig->MACAddr[6];
		configParam.MACAddr[7] = newConfig->MACAddr[7];

		// LEDs
		configParam.ir_LEDs_Enable = newConfig->ir_LEDs_Enable;
		configParam.visible_LEDs_Enable = newConfig->visible_LEDs_Enable;

		// sensor settings
		configParam.PIR0_Config.threshold = newConfig->PIR0_Config.threshold;
		configParam.PIR0_Config.blindTime_mS = newConfig->PIR0_Config.blindTime_mS;
		configParam.PIR0_Config.pulseCount = newConfig->PIR0_Config.pulseCount;
		configParam.PIR0_Config.windowTime_S = newConfig->PIR0_Config.windowTime_S;
		configParam.PIR0_Config.HPFCutOff = newConfig->PIR0_Config.HPFCutOff;
		configParam.PIR0_Config.countMode = newConfig->PIR0_Config.countMode;

		configParam.PIR1_Config.threshold = newConfig->PIR1_Config.threshold;
		configParam.PIR1_Config.blindTime_mS = newConfig->PIR1_Config.blindTime_mS;
		configParam.PIR1_Config.pulseCount = newConfig->PIR1_Config.pulseCount;
		configParam.PIR1_Config.windowTime_S = newConfig->PIR1_Config.windowTime_S;
		configParam.PIR1_Config.HPFCutOff = newConfig->PIR1_Config.HPFCutOff;
		configParam.PIR1_Config.countMode = newConfig->PIR1_Config.countMode;

		configParam.radarConfig.minRange_m = newConfig->radarConfig.minRange_m;
		configParam.radarConfig.maxRange_m = newConfig->radarConfig.maxRange_m;
		configParam.radarConfig.minVelocity_mps = newConfig->radarConfig.minVelocity_mps;
		configParam.radarConfig.maxVelocity_mps = newConfig->radarConfig.maxVelocity_mps;
		configParam.radarConfig.minSignal_dB = newConfig->radarConfig.minSignal_dB;
		configParam.radarConfig.maxSignal_dB = newConfig->radarConfig.maxSignal_dB;
		configParam.radarConfig.frequencyChannel = newConfig->radarConfig.frequencyChannel;
		configParam.radarConfig.falseAlarmSuppression = newConfig->radarConfig.falseAlarmSuppression;

		configParam.accelConfig.HPFSetting = newConfig->accelConfig.HPFSetting;
		configParam.accelConfig.LPFSetting = newConfig->accelConfig.LPFSetting;
		configParam.accelConfig.eventWindow_s = newConfig->accelConfig.eventWindow_s;
		configParam.accelConfig.eventCount = newConfig->accelConfig.eventCount;
		configParam.accelConfig.alarmWindow_s = newConfig->accelConfig.alarmWindow_s;
		configParam.accelConfig.supervisionTolerance = newConfig->accelConfig.supervisionTolerance;

		configParam.cameraConfig.decorationIPAddress = newConfig->cameraConfig.decorationIPAddress;
		configParam.cameraConfig.decorationMACAddress = newConfig->cameraConfig.decorationMACAddress;
		configParam.cameraConfig.decorationTimeOfDay = newConfig->cameraConfig.decorationTimeOfDay;
		configParam.cameraConfig.decorationNodeAddress = newConfig->cameraConfig.decorationNodeAddress;
		configParam.cameraConfig.enableMotionDetection = newConfig->cameraConfig.enableMotionDetection;
		configParam.cameraConfig.exposureControl = newConfig->cameraConfig.exposureControl;

		configParam.HFMEMSConfig.HPFSetting = newConfig->HFMEMSConfig.HPFSetting;
		configParam.HFMEMSConfig.LPFSetting = newConfig->HFMEMSConfig.LPFSetting;
		configParam.HFMEMSConfig.threshold = newConfig->HFMEMSConfig.threshold;
		configParam.HFMEMSConfig.LRInterior = newConfig->HFMEMSConfig.LRInterior;
		configParam.HFMEMSConfig.privacy = newConfig->HFMEMSConfig.privacy;

		configParam.warningBitfield = newConfig->warningBitfield;
		configParam.alarmBitfield = newConfig->alarmBitfield;


		// PW L3 items:
		configParam.cameraPrivacyEnabled = newConfig->cameraPrivacyEnabled;


		// PW L4 items:


		// PW L5 items:
		configParam.clearAccessLog = newConfig->clearAccessLog;


		// PW L6 items:
#endif

}


// newCfg is range checked up to the security level.
// beyond the security level, fall back config is used.
uint32_t sanitizeConfig(config_param_t *newCfgPtr, config_param_t *existingCfgPtr, uint32_t securityLevel)
{
	uint32_t fail;
	uint32_t i, bitmask;
	config_param_t tempCfg;
	memcpy(&tempCfg, existingCfgPtr, sizeof(tempCfg));


	do{

		// -- SECURITY LEVEL 1 ITEMS -- //
		if(securityLevel < 1){	break; }

		// -- SECURITY LEVEL 2 ITEMS -- //
		if(securityLevel < 2){	break; }
		memcpy(&tempCfg, newCfgPtr, ((uint32_t)(&tempCfg.cameraPrivacyEnabled)) - ((uint32_t)(&tempCfg))  );


		if(newCfgPtr->nodeAddress > 1000){newCfgPtr->nodeAddress = existingCfgPtr->nodeAddress;}

		if(newCfgPtr->GEO_latitude < (-90) || newCfgPtr->GEO_latitude > (90)){newCfgPtr->GEO_latitude = existingCfgPtr->GEO_latitude;}
		if(newCfgPtr->GEO_longitude < (-180) || newCfgPtr->GEO_longitude > (180)){newCfgPtr->GEO_longitude = existingCfgPtr->GEO_longitude;}

		if(newCfgPtr->mountingRoll < (-90) || newCfgPtr->mountingRoll > (90)){newCfgPtr->mountingRoll = existingCfgPtr->mountingRoll;}
		if(newCfgPtr->mountingPitch < (-90) || newCfgPtr->mountingPitch > (90)){newCfgPtr->mountingPitch = existingCfgPtr->mountingPitch;}


		fail = 0;
		if(newCfgPtr->ipAddress[0] == 0) { fail = TRUE; }
		if(newCfgPtr->ipAddress[0] >= 239){ fail = TRUE;}	// 240.x.x.x to 255.x.x.x are reserved.  239.x.x.x is reserved for multicast.
		if(newCfgPtr->ipAddress[0] == 224){ fail = TRUE;}	// 224.x.x.x is reserved for multicast.
		if(fail == TRUE){
			memcpy(&(newCfgPtr->ipAddress), &(existingCfgPtr->ipAddress), sizeof(newCfgPtr->ipAddress));
		}

		fail = 0;
		if(newCfgPtr->gwAddress[0] >= 239){ fail = TRUE;}	// 240.x.x.x to 255.x.x.x are reserved.  239.x.x.x is reserved for multicast.
		if(newCfgPtr->gwAddress[0] == 224){ fail = TRUE;}	// 224.x.x.x is reserved for multicast.
		if(fail == TRUE){
			memcpy(&(newCfgPtr->gwAddress), &(existingCfgPtr->gwAddress), sizeof(newCfgPtr->gwAddress));
		}

		bitmask = 0xFFFFFFFF;										// there are only 32 possible subnet masks.  check.
		for(i = 0; i < 32; i++){
			bitmask = (bitmask >> 1) & 0x7FFFFFFF;					// because of our endien-ness, we need to do this backwards.
			if( *((uint32_t *) newCfgPtr->snMask) == bitmask){
				break;
			}
		}
		if(i >= 32){
			memcpy(&(newCfgPtr->snMask), &(existingCfgPtr->snMask), sizeof(newCfgPtr->snMask));
		}

//		float_t 	sideA_DataSharing;
//		uint32_t 	sideA_DataSharingSetting;
//		float_t 	sideB_DataSharing;
//		uint32_t 	sideB_DataSharingSetting;
		if(newCfgPtr->sideA_DataSharing <= 0){newCfgPtr->sideA_DataSharing = existingCfgPtr->sideA_DataSharing;}
		if(newCfgPtr->sideB_DataSharing <= 0){newCfgPtr->sideB_DataSharing = existingCfgPtr->sideB_DataSharing;}

		if(newCfgPtr->ir_LEDs_Enable > 1){newCfgPtr->ir_LEDs_Enable = existingCfgPtr->ir_LEDs_Enable;}
		if(newCfgPtr->visible_LEDs_Enable > 1){newCfgPtr->visible_LEDs_Enable = existingCfgPtr->visible_LEDs_Enable;}
		if(newCfgPtr->accelConfig.HPFSetting > 8){newCfgPtr->accelConfig.HPFSetting  = existingCfgPtr->accelConfig.HPFSetting ;}
		if(newCfgPtr->accelConfig.LPFSetting > 8){newCfgPtr->accelConfig.LPFSetting  = existingCfgPtr->accelConfig.LPFSetting ;}
		if((newCfgPtr->accelConfig.eventWindow_s > 10) || (newCfgPtr->accelConfig.eventWindow_s < 0.5) || (fmod(newCfgPtr->accelConfig.eventWindow_s, 1) )) { newCfgPtr->accelConfig.eventWindow_s  = existingCfgPtr->accelConfig.eventWindow_s ;}
		if((newCfgPtr->accelConfig.eventCount > 10) || (newCfgPtr->accelConfig.eventCount < 1) ) {newCfgPtr->accelConfig.eventCount  = existingCfgPtr->accelConfig.eventCount ;}
		if((newCfgPtr->accelConfig.alarmWindow_s > 100) || (newCfgPtr->accelConfig.alarmWindow_s < 1) || (fmod(newCfgPtr->accelConfig.alarmWindow_s, 1))) {newCfgPtr->accelConfig.alarmWindow_s  = existingCfgPtr->accelConfig.alarmWindow_s ;}
		if(newCfgPtr->accelConfig.supervisionTolerance > 4){newCfgPtr->accelConfig.supervisionTolerance  = existingCfgPtr->accelConfig.supervisionTolerance ;}

		if((newCfgPtr->radarConfig.minRange_m > 50) || (newCfgPtr->radarConfig.minRange_m < 0) || ( fmod(newCfgPtr->radarConfig.minRange_m, 0.1) ) ){newCfgPtr->radarConfig.minRange_m  = existingCfgPtr->radarConfig.minRange_m ;}
		if((newCfgPtr->radarConfig.maxRange_m > 50) || (newCfgPtr->radarConfig.maxRange_m < 0) || ( fmod(newCfgPtr->radarConfig.maxRange_m, 0.1) ) ){newCfgPtr->radarConfig.maxRange_m  = existingCfgPtr->radarConfig.maxRange_m ;}
		if((newCfgPtr->radarConfig.minRange_m >= newCfgPtr->radarConfig.maxRange_m)){
			newCfgPtr->radarConfig.minRange_m  = existingCfgPtr->radarConfig.minRange_m ;
			newCfgPtr->radarConfig.maxRange_m  = existingCfgPtr->radarConfig.maxRange_m ;
		}

		if((newCfgPtr->radarConfig.minVelocity_mps > 50) || (newCfgPtr->radarConfig.minVelocity_mps < 0) || ( fmod(newCfgPtr->radarConfig.minVelocity_mps, 0.1) ) ){newCfgPtr->radarConfig.minVelocity_mps  = existingCfgPtr->radarConfig.minVelocity_mps ;}
		if((newCfgPtr->radarConfig.maxVelocity_mps > 50) || (newCfgPtr->radarConfig.maxVelocity_mps < 0) || ( fmod(newCfgPtr->radarConfig.maxVelocity_mps, 0.1) ) ){newCfgPtr->radarConfig.maxVelocity_mps  = existingCfgPtr->radarConfig.maxVelocity_mps ;}

		if((newCfgPtr->radarConfig.minVelocity_mps >= newCfgPtr->radarConfig.maxVelocity_mps)){
			newCfgPtr->radarConfig.minVelocity_mps  = existingCfgPtr->radarConfig.minVelocity_mps ;
			newCfgPtr->radarConfig.maxVelocity_mps  = existingCfgPtr->radarConfig.maxVelocity_mps ;
		}

		if((newCfgPtr->radarConfig.minSignal_dB > 50) || (newCfgPtr->radarConfig.minSignal_dB < 0) || ( fmod(newCfgPtr->radarConfig.minSignal_dB, 0.1) ) ){newCfgPtr->radarConfig.minSignal_dB  = existingCfgPtr->radarConfig.minSignal_dB ;}
		if((newCfgPtr->radarConfig.maxSignal_dB > 50) || (newCfgPtr->radarConfig.maxSignal_dB < 0) || ( fmod(newCfgPtr->radarConfig.maxSignal_dB, 0.1) ) ){newCfgPtr->radarConfig.maxSignal_dB  = existingCfgPtr->radarConfig.maxSignal_dB ;}

		if((newCfgPtr->radarConfig.minSignal_dB >= newCfgPtr->radarConfig.maxSignal_dB)){
			newCfgPtr->radarConfig.minSignal_dB  = existingCfgPtr->radarConfig.minSignal_dB ;
			newCfgPtr->radarConfig.maxSignal_dB  = existingCfgPtr->radarConfig.maxSignal_dB ;
		}

		if((newCfgPtr->radarConfig.frequencyChannel > 8) || (newCfgPtr->radarConfig.frequencyChannel < 0)  ){newCfgPtr->radarConfig.frequencyChannel  = existingCfgPtr->radarConfig.frequencyChannel ;}
		if((newCfgPtr->radarConfig.falseAlarmSuppression > 1)   ){ newCfgPtr->radarConfig.falseAlarmSuppression  = existingCfgPtr->radarConfig.falseAlarmSuppression ;}
		if((newCfgPtr->radarConfig.radarDistanceThreshold > 10) || (newCfgPtr->radarConfig.radarDistanceThreshold < 0.1)  ){	newCfgPtr->radarConfig.radarDistanceThreshold  = existingCfgPtr->radarConfig.radarDistanceThreshold ;}
		if((newCfgPtr->radarConfig.multiTargetTrackingSpaceing > 10) || (newCfgPtr->radarConfig.multiTargetTrackingSpaceing < 0.1)  ){	newCfgPtr->radarConfig.multiTargetTrackingSpaceing  = existingCfgPtr->radarConfig.multiTargetTrackingSpaceing ;}
		if((newCfgPtr->radarConfig.timeConstant >= MICROWAVE_KALMAN_BOXCAR_LEN) || (newCfgPtr->radarConfig.timeConstant < 1)  ){	newCfgPtr->radarConfig.timeConstant  = existingCfgPtr->radarConfig.timeConstant ;}

		if((newCfgPtr->PIR0_Config.threshold > 255) || (newCfgPtr->PIR0_Config.threshold < 0)  ){ newCfgPtr->PIR0_Config.threshold  = existingCfgPtr->PIR0_Config.threshold ;}
		if((newCfgPtr->PIR1_Config.threshold > 255) || (newCfgPtr->PIR1_Config.threshold < 0)  ){ newCfgPtr->PIR1_Config.threshold  = existingCfgPtr->PIR1_Config.threshold ;}
		if((newCfgPtr->PIR0_Config.blindTime_mS > 16) || (newCfgPtr->PIR0_Config.blindTime_mS < 1)  ){ newCfgPtr->PIR0_Config.blindTime_mS  = existingCfgPtr->PIR0_Config.blindTime_mS ;}
		if((newCfgPtr->PIR1_Config.blindTime_mS > 16) || (newCfgPtr->PIR1_Config.blindTime_mS < 1)  ){ newCfgPtr->PIR1_Config.blindTime_mS  = existingCfgPtr->PIR1_Config.blindTime_mS ;}
		if((newCfgPtr->PIR0_Config.pulseCount > 4) || (newCfgPtr->PIR0_Config.pulseCount < 1)  ){ newCfgPtr->PIR0_Config.pulseCount  = existingCfgPtr->PIR0_Config.pulseCount ;}
		if((newCfgPtr->PIR1_Config.pulseCount > 4) || (newCfgPtr->PIR1_Config.pulseCount < 1)  ){ newCfgPtr->PIR1_Config.pulseCount  = existingCfgPtr->PIR1_Config.pulseCount ;}
		if((newCfgPtr->PIR0_Config.windowTime_S > 4) || (newCfgPtr->PIR0_Config.windowTime_S < 2)  ){ newCfgPtr->PIR0_Config.windowTime_S  = existingCfgPtr->PIR0_Config.windowTime_S ;}
		if((newCfgPtr->PIR1_Config.windowTime_S > 4) || (newCfgPtr->PIR1_Config.windowTime_S < 2)  ){ newCfgPtr->PIR1_Config.windowTime_S  = existingCfgPtr->PIR1_Config.windowTime_S ;}
		if((newCfgPtr->PIR0_Config.HPFCutOff > 1) || (newCfgPtr->PIR0_Config.HPFCutOff < 0)  ){ newCfgPtr->PIR0_Config.HPFCutOff  = existingCfgPtr->PIR0_Config.HPFCutOff ;}
		if((newCfgPtr->PIR1_Config.HPFCutOff > 1) || (newCfgPtr->PIR1_Config.HPFCutOff < 0)  ){ newCfgPtr->PIR1_Config.HPFCutOff  = existingCfgPtr->PIR1_Config.HPFCutOff ;}
		if((newCfgPtr->PIR0_Config.countMode > 1) || (newCfgPtr->PIR0_Config.countMode < 0)  ){ newCfgPtr->PIR0_Config.countMode  = existingCfgPtr->PIR0_Config.countMode ;}
		if((newCfgPtr->PIR1_Config.countMode > 1) || (newCfgPtr->PIR1_Config.countMode < 0)  ){ newCfgPtr->PIR1_Config.countMode  = existingCfgPtr->PIR1_Config.countMode ;}

		if((newCfgPtr->cameraConfig.decorationIPAddress > 1) || (newCfgPtr->cameraConfig.decorationIPAddress < 0)  ){ newCfgPtr->cameraConfig.decorationIPAddress  = existingCfgPtr->cameraConfig.decorationIPAddress ;}
		if((newCfgPtr->cameraConfig.decorationMACAddress > 1) || (newCfgPtr->cameraConfig.decorationMACAddress < 0)  ){ newCfgPtr->cameraConfig.decorationMACAddress  = existingCfgPtr->cameraConfig.decorationMACAddress ;}
		if((newCfgPtr->cameraConfig.decorationTimeOfDay > 1) || (newCfgPtr->cameraConfig.decorationTimeOfDay < 0)  ){ newCfgPtr->cameraConfig.decorationTimeOfDay  = existingCfgPtr->cameraConfig.decorationTimeOfDay ;}
		if((newCfgPtr->cameraConfig.decorationNodeAddress > 1) || (newCfgPtr->cameraConfig.decorationNodeAddress < 0)  ){ newCfgPtr->cameraConfig.decorationNodeAddress  = existingCfgPtr->cameraConfig.decorationNodeAddress ;}
		if((newCfgPtr->cameraConfig.enableMotionDetection > 1) || (newCfgPtr->cameraConfig.enableMotionDetection < 0)  ){ newCfgPtr->cameraConfig.enableMotionDetection  = existingCfgPtr->cameraConfig.enableMotionDetection ;}
		if((newCfgPtr->cameraConfig.cameraImageSelection > 32) || (newCfgPtr->cameraConfig.cameraImageSelection < 0)  ){ newCfgPtr->cameraConfig.cameraImageSelection  = existingCfgPtr->cameraConfig.cameraImageSelection ;}

		if((newCfgPtr->HFMEMSConfig.HPFSetting > 1) || (newCfgPtr->HFMEMSConfig.HPFSetting < 0)  ){ newCfgPtr->HFMEMSConfig.HPFSetting  = existingCfgPtr->HFMEMSConfig.HPFSetting ;}
		if((newCfgPtr->HFMEMSConfig.LPFSetting > 1) || (newCfgPtr->HFMEMSConfig.LPFSetting < 0)  ){ newCfgPtr->HFMEMSConfig.LPFSetting  = existingCfgPtr->HFMEMSConfig.LPFSetting ;}
		if((newCfgPtr->HFMEMSConfig.threshold > 1000) || (newCfgPtr->HFMEMSConfig.threshold < 0)  ){ newCfgPtr->HFMEMSConfig.threshold  = existingCfgPtr->HFMEMSConfig.threshold ;}
		if((newCfgPtr->HFMEMSConfig.LRInterior > 8) || (newCfgPtr->HFMEMSConfig.LRInterior < 0)  ){ newCfgPtr->HFMEMSConfig.LRInterior  = existingCfgPtr->HFMEMSConfig.LRInterior ;}
		if((newCfgPtr->HFMEMSConfig.privacy > 1) || (newCfgPtr->HFMEMSConfig.privacy < 0)  ){ newCfgPtr->HFMEMSConfig.privacy  = existingCfgPtr->HFMEMSConfig.privacy ;}

		if((newCfgPtr->warningBitfield > 128) || (newCfgPtr->warningBitfield < 0)  ){ newCfgPtr->warningBitfield  = existingCfgPtr->warningBitfield ;}
		if((newCfgPtr->alarmBitfield > 128) || (newCfgPtr->alarmBitfield < 0)  ){ newCfgPtr->alarmBitfield  = existingCfgPtr->alarmBitfield ;}
		if((newCfgPtr->neighbourControl > 3) || (newCfgPtr->neighbourControl < 0)  ){ newCfgPtr->neighbourControl  = existingCfgPtr->neighbourControl ;}

		// -- SECURITY LEVEL 3 ITEMS -- //
		if(securityLevel < 3){	break; }


		// -- SECURITY LEVEL 4 ITEMS -- //
		if(securityLevel < 4){	break; }
		memcpy(&tempCfg.cameraPrivacyEnabled, newCfgPtr, ((uint32_t)(&tempCfg.clearAccessLog)) - ((uint32_t)(&tempCfg.cameraPrivacyEnabled))  );

		if((newCfgPtr->cameraPrivacyEnabled > 1) || (newCfgPtr->cameraPrivacyEnabled < 0)  ){ newCfgPtr->cameraPrivacyEnabled  = existingCfgPtr->cameraPrivacyEnabled ;}

	}while(0);// never loop.


	// some should always be stored as zero.
//	newCfgPtr->resetMountingAngle = 0;
	newCfgPtr->MACAddr[6] = 0;
	newCfgPtr->MACAddr[7] = 0;
	newCfgPtr->clearAccessLog = 0;
	// Sensor name is ASCII.  By definition, the last character needs to be 0x00.  or at least, there's no scenario where that isn't valid.
	newCfgPtr->sensorName[sizeof(newCfgPtr->sensorName) - 1] = 0;

	return 0;


}



// Sets configParam struct to defaults
// See app_config.h for parameter definitions/explanations
// all of these will be overwritten by flash if flash memory is found to be valid.
void config_init(void){
	//configParam.userPW[64]
	//configParam.serialNumber

	configParam.userPW[0] = 0;
	configParam.userPW[1] = 1;
	configParam.userPW[2] = 2;
	configParam.userPW[3] = 3;
	configParam.userPW[4] = 4;
	configParam.userPW[62] = 62;
	configParam.userPW[63] = 63;

	configParam.serialNumber = 0x0807060504030201;
	// PW L2 items:
	configParam.sensorName[0] = DEFAULT_NODE_NAME; // TODO
	configParam.nodeAddress = DEFAULT_NODE_ADDRESS;

	// Mounting Location
	configParam.GEO_latitude = DEFAULT_GEO_LATITUDE;
	configParam.GEO_longitude = DEFAULT_GEO_LONGITUDE;

	// Mounting Angle at time of installation
	configParam.mountingRoll = DEFAULT_MOUNTROLL;
	configParam.mountingPitch = DEFAULT_MOUNTPITCH;
//	configParam.mountingAngle_z = DEFAULT_MOUNTANGLE_Z;
//	configParam.resetMountingAngle = DEFAULT_MOUNTANGLE_RESET; // will reset to this value after change

	// NW settings
	configParam.ipAddress[0] = DEFAULT_NW_IPADDR0;
	configParam.ipAddress[1] = DEFAULT_NW_IPADDR1;
	configParam.ipAddress[2] = DEFAULT_NW_IPADDR2;
	configParam.ipAddress[3] = DEFAULT_NW_IPADDR3;
	configParam.snMask[0] = DEFAULT_NW_SUBNETMASK0;
	configParam.snMask[1] = DEFAULT_NW_SUBNETMASK1;
	configParam.snMask[2] = DEFAULT_NW_SUBNETMASK2;
	configParam.snMask[3] = DEFAULT_NW_SUBNETMASK3;
	configParam.gwAddress[0] = DEFAULT_NW_GATEWAYADDR0;
	configParam.gwAddress[1] = DEFAULT_NW_GATEWAYADDR1;
	configParam.gwAddress[2] = DEFAULT_NW_GATEWAYADDR2;
	configParam.gwAddress[3] = DEFAULT_NW_GATEWAYADDR3;
	configParam.IPBitField = DEFAULT_NW_SETTINGS_BITFLD;
	configParam.MACAddr[0] = DEFAULT_NW_MACADDR0;
	configParam.MACAddr[1] = DEFAULT_NW_MACADDR1;
	configParam.MACAddr[2] = DEFAULT_NW_MACADDR2;
	configParam.MACAddr[3] = DEFAULT_NW_MACADDR3;
	configParam.MACAddr[4] = DEFAULT_NW_MACADDR4;
	configParam.MACAddr[5] = DEFAULT_NW_MACADDR5;
	configParam.MACAddr[6] = DEFAULT_NW_MACADDR6;
	configParam.MACAddr[7] = DEFAULT_NW_MACADDR7;

	configParam.sideA_DataSharing = 1;
	configParam.sideA_DataSharingSetting = 0;
	configParam.sideB_DataSharing = 1;
	configParam.sideB_DataSharingSetting = 0;

	// LEDs
	configParam.ir_LEDs_Enable = DEFAULT_LED_IRENABLE;
	configParam.visible_LEDs_Enable = DEFAULT_LED_VISBILEENABLE;

	// sensor settings
	configParam.PIR0_Config.threshold = DEFAULT_PIR_THRESHOLD;
	configParam.PIR0_Config.blindTime_mS = DEFAULT_PIR_BLINDTIME;
	configParam.PIR0_Config.pulseCount = DEFAULT_PIR_PULSECOUNT;
	configParam.PIR0_Config.windowTime_S = DEFAULT_PIR_WINDOWTIME;
	configParam.PIR0_Config.HPFCutOff = DEFAULT_PIR_HPFCUTOFF;
	configParam.PIR0_Config.countMode = DEFAULT_PIR_COUNTMODE;

	configParam.PIR1_Config.threshold = DEFAULT_PIR_THRESHOLD;
	configParam.PIR1_Config.blindTime_mS = DEFAULT_PIR_BLINDTIME;
	configParam.PIR1_Config.pulseCount = DEFAULT_PIR_PULSECOUNT;
	configParam.PIR1_Config.windowTime_S = DEFAULT_PIR_WINDOWTIME;
	configParam.PIR1_Config.HPFCutOff = DEFAULT_PIR_HPFCUTOFF;
	configParam.PIR1_Config.countMode = DEFAULT_PIR_COUNTMODE;

	configParam.radarConfig.minRange_m = DEFAULT_RADAR_MINRANGE;
	configParam.radarConfig.maxRange_m = DEFAULT_RADAR_MAXRANGE;
	configParam.radarConfig.minVelocity_mps = DEFAULT_RADAR_MINVELOCITY;
	configParam.radarConfig.maxVelocity_mps = DEFAULT_RADAR_MAXVELOCITY;
	configParam.radarConfig.minSignal_dB = DEFAULT_RADAR_MINSIGNAL;
	configParam.radarConfig.maxSignal_dB = DEFAULT_RADAR_MAXSIGNAL;
	configParam.radarConfig.frequencyChannel = DEFAULT_RADAR_FREQCHANNEL;
	configParam.radarConfig.falseAlarmSuppression = DEFAULT_RADAR_FALSEALARMSURPRESSION;
	configParam.radarConfig.radarDistanceThreshold = DEFAULT_RADAR_DISTANCETHRESHOLD;
	configParam.radarConfig.multiTargetTrackingSpaceing = DEFAULT_RADAR_MULTITARGETTRACKINGSPACING;
	configParam.radarConfig.timeConstant = DEFAULT_RADAR_TIMECONSTANT;


	configParam.accelConfig.HPFSetting = DEFAULT_ACCEL_HPFSETTING;
	configParam.accelConfig.LPFSetting = DEFAULT_ACCEL_LPFSETTING;
	configParam.accelConfig.eventWindow_s = DEFAULT_ACCEL_EVENTWINDOW;
	configParam.accelConfig.eventCount = DEFAULT_ACCEL_EVENTCOUNT;
	configParam.accelConfig.alarmWindow_s = DEFAULT_ACCEL_ALARMWINDOW;
	configParam.accelConfig.supervisionTolerance = DEFAULT_ACCEL_SUPERVISIONTOLERANCE;

	configParam.cameraConfig.decorationIPAddress = DEFAULT_CAMERA_IPDECORATION;
	configParam.cameraConfig.decorationMACAddress = DEFAULT_CAMERA_MACDECORATION;
	configParam.cameraConfig.decorationTimeOfDay = DEFAULT_CAMERA_TIMEDECORATION;
	configParam.cameraConfig.decorationNodeAddress = DEFAULT_CAMERA_NODEADDRDECORATION;
	configParam.cameraConfig.enableMotionDetection = DEFAULT_CAMERA_MOTIONDETECTION;
	configParam.cameraConfig.exposureControl = DEFAULT_CAMERA_EXPOSURECTRL;
	configParam.cameraConfig.cameraImageSelection = DEFAULT_CAMERA_IMAGESELECTION;

	configParam.HFMEMSConfig.HPFSetting = DEFAULT_HFMEMS_HPFSETTING;
	configParam.HFMEMSConfig.LPFSetting = DEFAULT_HFMEMS_LPFSETTING;
	configParam.HFMEMSConfig.threshold = DEFAULT_HFMEMS_THRESHOLD;
	configParam.HFMEMSConfig.LRInterior = DEFAULT_HFMEMS_LRINTERIOR;
	configParam.HFMEMSConfig.privacy = DEFAULT_HFMEMS_PRIVACY;

	configParam.warningBitfield = DEFAULT_WARNING_BITFLD;
	configParam.alarmBitfield = DEFAULT_ALARM_BITFLD;


	// PW L3 items:
	configParam.cameraPrivacyEnabled = DEFAULT_PRIVACY_CAMERA;


	// PW L4 items:


	// PW L5 items:
	configParam.clearAccessLog = DEFAULT_ACCESSLOG_CLEARVAL;


	// PW L6 items:

}

// Sets configParam struct to version stored in flash
void config_reinit(void){
	// not yet implemented
}

void app_config_task(void *pvParameters){
	uint8_t hasValidConfig = 0;

	if(APP_FLASH_ENABLED){
		// read flash for config
	}
	// valid config found
	if(hasValidConfig){
		config_reinit();
	}
	else{
		// set config to default
		config_init();
		diagnostic_flags |= JACK_DIAG_STATUS_DEFAULTCFG_MASK;
	}
	xEventGroupSetBits(configEventGroup, CONFIGEVENT_CONFIGRDY|CONFIGEVENT_NWRKRDY); // notify sensor & network tasks that the config is set

	while(1){
		// await config changes from jack
		ulTaskNotifyTake(pdTRUE, ( TickType_t ) portMAX_DELAY);
		// save new config to flash and update configParam
	}
}
