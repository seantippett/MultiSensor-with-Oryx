#ifndef __app_jack_h
#define __app_jack_h

#include "math.h"
#include "app_video.h"
#include "app_shared.h"
#include "app_config.h"

extern void SwTimerCallback_Jack(TimerHandle_t xTimer);
extern TimerHandle_t SwTimerHandle_JackTCPIP;
extern SemaphoreHandle_t xJackMutex;
extern SemaphoreHandle_t xJackConnectionMutex;


// Debug Defines
#define JACK_SEND_IMAGES					1


#define STX_VAL 							0x000034E0
#define JACK_MSG_GENERIC_VER				0
#define JACK_HEADER_SIZE 					(4*4) // STX LEN TYPE VER
#define JACK_TIMECRITICAL_SUBHEADER_SIZE 	(4+8) // NODE TIME(u64)

#define SW_TIMER_PERIOD_JACK_MS	500

#define JACK_MSG_REQUEST_MASK 	0x80000000

// Time Critical Application Messages - Payload size in bytes
#define JACK_MSG_TYPE_APP_NULL		0x00010001
#define JACK_MSG_SIZE_APP_NULL		32
#define JACK_MSG_SIZE_APP_NULL_REQ	32
#define JACK_MSG_VER_APP_NULL		JACK_MSG_GENERIC_VER

#define JACK_MSG_TYPE_APP_CONF		0x00010002
#define JACK_MSG_SIZE_APP_CONF		62
#define JACK_MSG_SIZE_APP_CONF_REQ	32

#define JACK_MSG_TYPE_APP_AV		0x00010004
#define JACK_MSG_SIZE_APP_AV		0 // not yet determined
#define JACK_MSG_SIZE_APP_AV_REQ	32

#define JACK_MSG_TYPE_APP_GSD		0x00010005
#define JACK_MSG_SIZE_APP_GSD		131072 // 128kB
#define JACK_MSG_SIZE_APP_GSD_REQ	8

#define JACK_MSG_TYPE_APP_AUDIO		0x00010006
#define JACK_MSG_SIZE_APP_AUDIO		48000
#define JACK_MSG_SIZE_APP_AUDIO_REQ	4

#define JACK_MSG_TYPE_APP_ALARM		0x00010007
#define JACK_MSG_SIZE_APP_ALARM		8
#define JACK_MSG_SIZE_APP_ALARM_REQ	4
#define JACK_MSG_VER_APP_ALARM		JACK_MSG_GENERIC_VER

#define JACK_MSG_TYPE_APP_UCM_PLOT		0x00010013	//0x00010003
#define JACK_MSG_SIZE_APP_UCM_PLOT		24644
#define JACK_MSG_SIZE_APP_UCM_PLOT_REQ	4
#define JACK_MSG_VER_APP_UCM_PLOT		JACK_MSG_GENERIC_VER


// Housekeeping Messages - Payload (not headers or subheaders [optional subtype is not part of a subheader]) size in bytes
#define JACK_MSG_TYPE_NULL				0x00000000
#define JACK_MSG_SIZE_NULL				32
#define JACK_MSG_SIZE_NULL_REQ			32
#define JACK_MSG_VER_NULL				JACK_MSG_GENERIC_VER
#define JACK_MSG_TYPE_DEVICEID			0x00000001
#define JACK_MSG_SIZE_DEVICEID			32
#define JACK_MSG_SIZE_DEVICEID_REQ		0
#define JACK_MSG_VER_DEVICEID			JACK_MSG_GENERIC_VER
#define JACK_MSG_TYPE_REBOOT			0x00000002
#define JACK_MSG_SIZE_REBOOT			8
#define JACK_MSG_SIZE_REBOOT_REQ		8
#define JACK_MSG_VER_REBOOT				JACK_MSG_GENERIC_VER
#define JACK_MSG_TYPE_RAWHIST			0x00000003
#define JACK_MSG_SIZE_RAWHIST			1012000
#define JACK_MSG_SIZE_RAWHIST_REQ		4
#define JACK_MSG_TYPE_FLASH				0x00000004
#define JACK_MSG_SIZE_FLASH				4
#define JACK_MSG_SIZE_FLASH_REQ			4
#define JACK_MSG_TYPE_HWMAGNITUDE		0x00000005
#define JACK_MSG_SIZE_HWMAGNITUDE		32
#define JACK_MSG_SIZE_HWMAGNITUDE_REQ	8
#define JACK_MSG_VER_HWMAGNITUDE		JACK_MSG_GENERIC_VER
#define JACK_MSG_TYPE_FWUPDATE 			0x00000006
#define JACK_MSG_SIZE_FWUPDATE 			8
#define JACK_MSG_SIZE_FWUPDATE_REQ 		8
#define JACK_MSG_VER_FWUPDATE 			JACK_MSG_GENERIC_VER
#define JACK_MSG_TYPE_DEVDSCRIP			0x0000000A
#define JACK_MSG_SIZE_DEVDSCRIP			260
#define JACK_MSG_SIZE_DEVDSCRIP_REQ		260
#define JACK_MSG_TYPE_DEVICEIP			0x0000000B
#define JACK_MSG_SIZE_DEVICEIP			32
#define JACK_MSG_SIZE_DEVICEIP_REQ		32
#define JACK_MSG_TYPE_CONFIG			0x0000000C
#define JACK_MSG_SIZE_CONFIG			520
#define JACK_MSG_SIZE_CONFIG_REQ		524
#define JACK_MSG_VER_CONFIG				JACK_MSG_GENERIC_VER
#define JACK_MSG_TYPE_HISTORY			0x0000000D
#define JACK_MSG_SIZE_HISTORY			32
#define JACK_MSG_SIZE_HISTORY_REQ		32
#define JACK_MSG_VER_HISTORY			JACK_MSG_GENERIC_VER
#define JACK_MSG_TYPE_POWER				0x0000000E
#define JACK_MSG_SIZE_POWER				24
#define JACK_MSG_SIZE_POWER_REQ			4
#define JACK_MSG_VER_POWER				JACK_MSG_GENERIC_VER
#define JACK_MSG_TYPE_DIAG				0x0000000F
#define JACK_MSG_SIZE_DIAG				16
#define JACK_MSG_SIZE_DIAG_REQ			4
#define JACK_MSG_VER_DIAG				JACK_MSG_GENERIC_VER
#define JACK_MSG_TYPE_LIVESETTINGS		0x00000010
#define JACK_MSG_SIZE_LIVESETTINGS		(sizeof(jack_msg_livesettings_t) - sizeof(jack_header_t))
#define JACK_MSG_SIZE_LIVESETTINGS_REQ	(sizeof(jack_msg_liveconfig_req_t) - sizeof(jack_header_t))
#define JACK_MSG_VER_LIVESETTINGS		JACK_MSG_GENERIC_VER

// Misc. Defines
#define JACK_DEVICEID_NEWRESET_MASK 0x80000000
#define JACK_REBOOT_RESET_VAL		0xFF00FF00
#define JACK_POWER_SUBTYPE_1		0x00000001

// Plot Message Defines
#define JACK_APP_PLOT_STREAM_STOP			0x00000000
#define JACK_APP_PLOT_STREAM_START			0x00000001
#define JACK_APP_PLOT_STREAM_SEND1			0x00000002

// Sensor Alarm Defines
#define JACK_APP_ALARM_SUBTYPE_1			0x00000001
#define JACK_APP_ALARM_ACCEL_MASK			0x00000001
#define JACK_APP_ALARM_RADAR_MASK			0x00000002
#define JACK_APP_ALARM_LPIR_MASK			0x00000004
#define JACK_APP_ALARM_RPIR_MASK			0x00000008
#define JACK_APP_ALARM_HFMEMS_MASK			0x00000010
#define JACK_APP_ALARM_CAMERA_MASK			0x00000020
#define JACK_APP_ALARM_SWITCH1TAMPER_MASK	0x00000040
#define JACK_APP_ALARM_SWITCH2TAMPER_MASK	0x00000080
#define JACK_APP_ALARM_COMBINATION_MASK		0x00000100

// Device History Defines
#define	JACK_HISTORY_CONTROL_RESETPOWERFLTS		0x00000001
#define	JACK_HISTORY_CONTROL_RESETNETWORKFLTS	0x00000002
#define	JACK_HISTORY_CONTROL_RESETSENSORFLTS	0x00000004


// Diagnostic Defines
#define JACK_DIAG_SUBTYPE_1						0x00000001

#define JACK_DIAG_STATUS_TAMPER_MASK			0x00000001
#define JACK_DIAG_STATUS_INTERNALERR_MASK		0x00000002
#define JACK_DIAG_STATUS_1588SYNC_MASK			0x00000004
#define JACK_DIAG_STATUS_RESERVED1_MASK			0x00000008
#define JACK_DIAG_STATUS_RESERVED2_MASK			0x00000010
#define JACK_DIAG_STATUS_DEFAULTCFG_MASK		0x00000020
#define JACK_DIAG_STATUS_RESERVED3_MASK			0x00000040
#define JACK_DIAG_STATUS_RESERVED4_MASK			0x00000080
#define JACK_DIAG_STATUS_ACCELFAULT_MASK		0x00000100
#define JACK_DIAG_STATUS_ACCELOFFLINE_MASK		0x00000200
#define JACK_DIAG_STATUS_LPIRFAULT_MASK			0x00000400
#define JACK_DIAG_STATUS_LPIROFFLINE_MASK		0x00000800
#define JACK_DIAG_STATUS_RPIRFAULT_MASK			0x00001000
#define JACK_DIAG_STATUS_RPIROFFLINE_MASK		0x00002000
#define JACK_DIAG_STATUS_RADARFAULT_MASK		0x00004000
#define JACK_DIAG_STATUS_RADAROFFLINE_MASK		0x00008000
#define JACK_DIAG_STATUS_HFMEMSFAULT_MASK		0x00010000
#define JACK_DIAG_STATUS_HFMEMSOFFLINE_MASK		0x00020000
#define JACK_DIAG_STATUS_CAMFAULT_MASK			0x00040000
#define JACK_DIAG_STATUS_CAMOFFLINE_MASK		0x00080000

#define JACK_DIAG_POWER_INPUTFAULT_MASK			0x00000001
#define JACK_DIAG_POWER_RESERVED1_MASK			0x00000002
#define JACK_DIAG_POWER_RESERVED2_MASK			0x00000004
#define JACK_DIAG_POWER_RESERVED3_MASK			0x00000008

#define JACK_DIAG_NETWORK_SIDEAFAIL_MASK		0x00000001
#define JACK_DIAG_NETWORK_SIDEBFAIL_MASK		0x00000002
#define JACK_DIAG_NETWORK_RESERVED1_MASK		0x00000004
#define JACK_DIAG_NETWORK_RESERVED2_MASK		0x00000008

// Live Settings Defines
#define JACK_LIVESETTINGS_VIDDECOR_TIME				0x00000001
#define JACK_LIVESETTINGS_VIDDECOR_NODE				0x00000002
#define JACK_LIVESETTINGS_VIDDECOR_IPADDR			0x00000004
#define JACK_LIVESETTINGS_VIDDECOR_TRACKING			0x00000008
#define JACK_LIVESETTINGS_VIDDECOR_TRACKINGBOX		0x00000010



// Jack Stream Defines
#define JACK_STREAM_PLOT_MASK					0x00000001

//#define JACK_STREAM_BASEDELAY_MS				100
//#define JACK_STREAM_BASERATE_HZ					10
//#define JACK_STREAM_2Hz(x)						(x % JACK_STREAM_BASEDELAY) \\ this future-proofing isn't strictly necessary



// Message Structs
#pragma pack(1)
typedef struct STR_Jack_Header{
	uint32_t	STX;
	uint32_t	LEN;
	uint32_t	TYPE;
	uint32_t	VER;
} jack_header_t;

#pragma pack(1)
typedef struct STR_Jack_TimeCritical_Subheader{
	uint32_t	NODE;
	uint64_t	TIME;
} jack_timeCritical_subheader_t;

#pragma pack(1)
struct STR_Jack_RadarMsg_cartesian{
	uint8_t		xData;
	uint8_t		yData;
	uint8_t		confidence;
	uint8_t		magnitude;
};


#pragma pack(1)
union U_Jack_RadarMsg_cartesian{
	struct STR_Jack_RadarMsg_cartesian message;
	uint32_t raw;
};

#pragma pack(1)
struct STR_Jack_PIR{
	uint16_t confidence;
	uint16_t magnitude;
};

#pragma pack(1)
struct STR_Jack_Accel{
	uint16_t confidence;
	uint16_t magnitude;
};


// Application / Time Critical Messages - Replies and Requests
#pragma pack(1)
typedef struct STR_Jack_MSG_1_1{ // app null
	jack_header_t header;
	jack_timeCritical_subheader_t subheader;
	uint32_t data[8];
} jack_msg_app_null_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_1_1{ // app null request
	jack_header_t header;
	uint32_t data[8];
} jack_msg_app_null_req_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_1_2 { // confidence
	jack_header_t header;
	jack_timeCritical_subheader_t subheader;
	union U_Jack_RadarMsg_cartesian radarMessage_cartesian[10];
	struct STR_Jack_PIR pirMessage[2];
	struct STR_Jack_Accel accelMessage;
	uint32_t    cameraData[2];
} jack_msg_app_confidence_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_1_3{ // UCM plot
	jack_header_t header;
	jack_timeCritical_subheader_t subheader;
	uint32_t accelMagnitude;	//
	uint32_t leftPIRMagnitude;	//
	uint32_t rightPIRMagnitude;	//
	uint32_t HFMEMSMagnitude;	//
	uint32_t cameraMagnitude;	//
	uint32_t radarMagnitude;	//
	float radarXYMagnitude[5][3];		// x,y, magnitude. x,y, magnitude., x,y, magnitude...

	uint32_t	jpegSize;
	uint8_t		*jpegData;		// [JPG_IMAGE_BUFFER_SIZE];
} jack_msg_app_ucm_plot_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_1_3{ // plot request
	jack_header_t header;
	uint32_t streamingControl;
} jack_msg_app_plot_req_t;
#pragma pack(0)

typedef struct STR_Jack_MSG_1_4{ // audio/video
	jack_header_t header;
	jack_timeCritical_subheader_t subheader;
	uint32_t data[8];
} jack_msg_app_AV_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_1_5{ // gunshot detection
	jack_header_t header;
	jack_timeCritical_subheader_t subheader;
	uint32_t data[8];
} jack_msg_app_gunshot_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_1_6{ // audio
	jack_header_t header;
	jack_timeCritical_subheader_t subheader;
	uint32_t data[8];
} jack_msg_app_audio_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_1_7{ // sensor alarm
	jack_header_t header;
	jack_timeCritical_subheader_t subheader;
	uint32_t SUBTYPE;
	uint32_t alarm_bitfield;
} jack_msg_app_alarm_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_1_7{ // sensor alarm request
	jack_header_t header;
	uint32_t requestSubtype;
} jack_msg_app_alarm_req_t;


// Housekeeping / Common Messages - Replies and Requests
#pragma pack(1)
typedef struct STR_Jack_MSG_0_0 { // null
	jack_header_t header;
//	uint32_t    data[8];
} jack_msg_null_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_0_0 { // null request
	jack_header_t header;
//	uint32_t    data[8];
} jack_msg_null_req_t;
#pragma pack(0)


#pragma pack(1)
typedef struct STR_Jack_MSG_0_1 { // device ID
	jack_header_t header;
	uint32_t	deviceType;
	uint32_t	fwVersion;
	uint32_t	reserved1;
	uint32_t	serialNumber;
	uint32_t	reserved2;
	uint32_t	hwVersion;
	uint32_t	reserved3;
	uint32_t    resetCause;
} jack_msg_deviceID_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_0_1 { // device ID request
	jack_header_t header;
	// no payload
} jack_msg_deviceID_req_t;
#pragma pack(0)


#pragma pack(1)
typedef struct STR_Jack_MSG_0_2 { // reboot
	jack_header_t header;
	uint32_t    resetReply[2];
} jack_msg_reboot_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_0_2 { // reboot request
	jack_header_t header;
	uint32_t    resetRequest[2];
} jack_msg_reboot_req_t;
#pragma pack(0)


#pragma pack(1)
typedef struct STR_Jack_MSG_0_3 { // raw history -- stream
	jack_header_t header;
	uint32_t    data[32];
} jack_msg_rawhistory_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_0_4 { // flash
	jack_header_t header;
	uint32_t    data[32];
} jack_msg_flash_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_0_5 { // hardware magnitude
	jack_header_t header;
	uint32_t    data[32];
} jack_msg_hardwaremagnitude_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_0_6 { // FW Update
	jack_header_t header;
	uint32_t 	SUBTYPE;
	union {
		struct	{
			uint32_t	address;
		}subtype1;
		struct	{
			uint16_t	crcChecksum;
		}subtype2;

	};
} jack_msg_FWupdate_t;

#pragma pack(1)
struct STR_JACK_MagConfidence{
	uint8_t		magnitude;
	uint8_t		confidence;
};


#pragma pack(1)
struct STR_Jack_RadarMsg_polar{
	int8_t		angle;
	uint8_t		distance;
	uint8_t		magnitude;
};

#pragma pack(1)
struct STR_Jack_CameraTarget{
	uint8_t 	boxTopLeft_X;
	uint8_t		boxTopLeft_Y;
	uint8_t		box_w;
	uint8_t		box_h;
	uint16_t	backgroundIntensity;
	uint16_t	varianceIntensity;
};

#pragma pack(1)
struct STR_Jack_MSG_2_3 {	// jack msg 2.3 - Plot data for sensors. - logging msg uses this.
	uint32_t	STX;
	uint32_t	LEN;
	uint32_t	TYPE;
	uint32_t	VER;
	uint32_t	NODE;
	uint64_t	TIME;

	union U_Jack_RadarMsg_cartesian radarMessage_cartesian[10];

	struct STR_JACK_MagConfidence leftPIRMagConf;
	struct STR_JACK_MagConfidence rightPIRMagConf;
	struct STR_JACK_MagConfidence accelMagConf;
	struct STR_JACK_MagConfidence cameraMagConf;

	uint16_t	temperature;
	uint8_t		packingByte[2];

	struct STR_Jack_RadarMsg_polar rawRadar[80];

	uint8_t		leftPir[40];
	uint8_t		rightPir[40];
	int16_t    accelData[4800];
	struct STR_Jack_CameraTarget cameraTargets[80];	
	uint32_t	jpegSize;
	uint8_t		jpegData[JPG_IMAGE_BUFFER_SIZE];
} ;






#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_0_6 { // FW Update request
	jack_header_t header;
	uint32_t 	SUBTYPE;
	union {
		struct {
			uint32_t	address;
			uint8_t		data[512];
		}subtype1;
		struct {
			uint32_t	imageSize;
			uint16_t	crcChecksum;
		}subtype2;

	};
} jack_msg_FWupdate_req_t;





#pragma pack(1)
typedef struct STR_Jack_MSG_0_A { // device description
	jack_header_t header;
	uint32_t    data[32];
} jack_msg_devdescription_t;

//#pragma pack(1)		-- This msg no longer exists.
//typedef struct STR_Jack_MSG_0_B { // device IP
//	jack_header_t header;
//	uint32_t    data[32];
//} jack_msg_deviceIP_t;

#pragma pack(1)
typedef struct STR_Jack_MSG_0_C { // config
	jack_header_t header;
	config_param_t configParamaters;
} jack_msg_config_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_0_C { // config request
	jack_header_t header;
	uint32_t setConfig;
	config_param_t configParamaters;
} jack_msg_config_req_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_0_D { // device history
	jack_header_t header;
	uint32_t	faults6v5;
	uint32_t	faults3v3;
	uint32_t	faultsSideAPower;
	uint32_t	faultsSideBPower;
	uint32_t	faultsSideANetwork;
	uint32_t	faultsSideBNetwork;
	uint32_t	faultsAccel;
	uint32_t	faultsLPIR;
	uint32_t	faultsRPIR;
	uint32_t	faultsRadar;
	uint32_t	faultsHFMEMS;
	uint32_t	faultsCamera;
} jack_msg_devicehistory_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_0_D { // device history request
	jack_header_t header;
	uint32_t    historyControl;
} jack_msg_devicehistory_req_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_0_E { // power
	jack_header_t header;
	uint32_t	SUBTYPE;
	float_t		inputVoltage;
	float_t		sideAcurrent;
	float_t		sideBcurrent;
	uint32_t	powerState;
	uint32_t	flags;
	float_t		roll;
	float_t		pitch;
} jack_msg_power_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_0_E { // power request
	jack_header_t header;
	uint32_t    requestSubtype;
} jack_msg_power_req_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_0_F { // diagnostics
	jack_header_t header;
	uint32_t	SUBTYPE;
	uint32_t	diagnosticStatusFlags;
	uint32_t	voltageRailFaults;
	uint32_t	networkFaults;
} jack_msg_diag_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_0_F { // diagnostics request
	jack_header_t header;
	uint32_t    requestSubtype;
} jack_msg_diag_req_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_0_10 { // live settings
	jack_header_t header;
	uint32_t videoDecorations;
	uint32_t imageProcessingPreviewSelect;
} jack_msg_livesettings_t;
#pragma pack(0)

#pragma pack(1)
typedef struct STR_Jack_MSG_REQ_0_10 { // live settings request
	jack_header_t header;
	uint32_t videoDecorations;
	uint32_t imageProcessingPreviewSelect;
} jack_msg_livesettings_req_t;
#pragma pack(0)

extern uint8_t diagnostic_change_flag; // set by sensors and cleared when message is sent
void refreshDiagnosticFlags(void);
extern uint32_t diagnostic_flags;
extern uint32_t diagnostic_flagsSent;
extern bool tamper_flag;


extern uint16_t accelDataForJack;
extern uint32_t accelConfidenceForJack;
extern uint32_t accelConfidenceForLogging;
extern union U_Jack_RadarMsg_cartesian	radarDataForJack_cartesian[10];
extern struct STR_Jack_RadarMsg_polar	radarDataForJack_polar[10];
extern uint32_t radarConfidenceForJack;
extern uint32_t radarConfidenceForLogging;
extern uint16_t pirDataForJack[2];
extern uint32_t pirConfidenceForJack;
extern uint32_t pirConfidenceForLogging;
void jackServerTask(void *pvParameters);





#endif
