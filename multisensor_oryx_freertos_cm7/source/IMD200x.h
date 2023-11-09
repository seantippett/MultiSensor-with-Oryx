



#ifndef _IMD200x_H
#define _IMD200x_H

#include "app_config.h"

typedef enum {
	IMD2002_API_ERR_OK = 0x0000,
	IMD2002_API_ERR_FUNCTION_DEPRECATED,
	IMD2002_API_ERR_DLL_NOT_FINISHED,
	IMD2002_API_ERR_HANDLE_NOT_INITIALIZED,
	IMD2002_API_ERR_COMPORT_DOESNT_EXIST,
	IMD2002_API_ERR_COMPORT_CANT_INITIALIZE,
	IMD2002_API_ERR_COMPORT_ACCESS_DENIED,
	IMD2002_API_ERR_COMPORT_BAUDRATE_NOT_VALID,
	IMD2002_API_ERR_COMPORT_CANT_OPEN ,
	IMD2002_API_ERR_COMPORT_CANT_SET_FLOW_CONTROL,
	IMD2002_API_ERR_COMPORT_CANT_SET_PARITY,
	IMD2002_API_ERR_COMPORT_CANT_SET_STOP_BITS,
	IMD2002_API_ERR_COMPORT_CANT_SET_DATA_BITS,
	IMD2002_API_ERR_COMPORT_CANT_SET_BAUDRATE,
	IMD2002_API_ERR_COMPORT_ALREADY_INITIALIZED ,
	IMD2002_API_ERR_COMPORT_EQUALS_NULL,
	IMD2002_API_ERR_COMPORT_NOT_OPEN ,
	IMD2002_API_ERR_COMPORT_NOT_READABLE ,
	IMD2002_API_ERR_COMPORT_NOT_WRITEABLE ,
	IMD2002_API_ERR_COMPORT_CANT_WRITE ,
	IMD2002_API_ERR_COMPORT_CANT_READ ,
	IMD2002_API_ERR_COMMAND_NOT_WRITTEN ,
	IMD2002_API_ERR_COMMAND_NOT_READ ,
	IMD2002_API_ERR_COMMAND_NO_DATA_RECEIVED,
	IMD2002_API_ERR_COMMAND_NO_VALID_FRAME_FOUND,
	IMD2002_API_ERR_COMMAND_RX_FRAME_DAMAGED,
	IMD2002_API_ERR_COMMAND_FAILURE,
	IMD2002_API_ERR_UNDEFINED_READ ,
	IMD2002_API_ERR_COMPORT_LESS_DATA_READ ,
	IMD2002_API_ERR_COMPORT_SYSTEM_INIT_FAILED,
	IMD2002_API_ERR_COMPORT_SYSTEM_ALREADY_INITIALIZED ,
	IMD2002_API_ERR_COMMAND_RX_FRAME_LENGTH,
	IMD2002_API_ERR_COMMAND_MAX_DATA_OVERFLOW,
	IMD2002_API_ERR_COMMAND_MAX_IQPAIRS_OVERFLOW,
	IMD2002_API_ERR_COMMAND_NOT_ACCEPTED,
	IMD2002_API_ERR_NULL_POINTER ,
	IMD2002_API_ERR_PARAMETER_OUT_OF_RANGE ,
	IMD2002_API_ERR_COMMAND_UNEXPECTED_FRAMETYPE,
	IMD2002_API_ERR_COMMAND_WITH_FAILURE_CODE
} IMD2002_Result_t;

typedef struct {
	float f32_range_m;
	float f32_velocity_mps;
	float f32_signal_dB;
	float f32_estimatedTimeOfArrival_s;
	float f32_incidentAngle_deg;

}IMD2002_Target_t;

typedef struct {
	float f32_range_m;
	float f32_velocity_mps;
	float f32_signal_dB;
	float f32_estimatedTimeOfArrival_s;
}IMD2000_Target_t;


#define IMD2002_MAX_TARGETS (15)
#define IMD2000_MAX_TARGETS (20)

typedef struct {
	uint16_t ui16_nrOfTargets;
	uint16_t ui16_targetListId;
	uint16_t ui16_reserved1;
	uint16_t ui16_reserved2;
	IMD2002_Target_t target[IMD2002_MAX_TARGETS];
}IMD2002_TargetList_t;

typedef struct {
	uint16_t ui16_nrOfTargets;
	uint16_t ui16_targetListId;
	uint16_t ui16_reserved1;
	uint16_t ui16_reserved2;
	IMD2000_Target_t target[IMD2000_MAX_TARGETS];
}IMD2000_TargetList_t;

extern IMD2002_TargetList_t IMD2002TargetList;
extern IMD2000_TargetList_t IMD2000TargetList;
extern uint32_t	acquisitionRunning;



union U_LOWER_UPPER{
	struct {
		float lower;
		float upper;
	};
	uint8_t raw8[8];
	uint32_t raw32[2];
};


#pragma pack(1)
struct IMD200x_settings {
	union U_LOWER_UPPER	velocity;
	union U_LOWER_UPPER	range;
	union U_LOWER_UPPER	signal;

	uint16_t	falseAlarmSuppression;
	uint16_t	freqChannel;
	uint32_t	serialNumber;
	uint32_t	productInfo;
};
//extern struct IMD200x_settings defaultIMD200xSettings;
extern struct IMD200x_settings IMD200xSettings ;
#pragma pack()

uint8_t IMD2002_init( struct IMD200x_settings *newSettings );
void requestNewTargetList(void);
void startAcquisition(void);
void stopAcquisition(void);
void flushUART(void);
uint32_t receiveNewTargetList(void);

void setRadarConfig(struct STR_radarConfig *newConfig);
void getRadarConfig(struct STR_radarConfig *activeConfig);

#endif
