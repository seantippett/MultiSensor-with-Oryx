/*
 * app_Jack.c
 *
 *  Created on: Jun. 28, 2022
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

#include "app_shared.h"
//#include "app_shield.h"

//#include "McuWait.h"
//#include "McuRTOS.h"



//Oryx
#include "core/net.h"
#include "tls.h"
#include "tls_cipher_suites.h"
#include "tls_ticket.h"
#include "tls_misc.h"
#include "rng/yarrow.h"
#include "resource_manager.h"
#include "debug.h"
#include "drivers/switch/ksz9563_driver.h"

#include "board.h"
#include "app_microwave.h"
#include "IMD200x.h"
#include "app_jack.h"
#include "app_board.h"
#include "app_shared.h"
#include "app_PIR.h"
#include "app_accel.h"
#include "app_microwave.h"
#include "app_logging.h"
#include "app_video.h"
#include "senstarCRC.h"
#include "simpleFlashSupport.h"
#include "task.h"
#include "phy_ksz9563.h"


#define JACK_BUFFER_SIZE	(1460 * 6)
#define PORT                        10013

//Application configuration
#define APP_SERVER_PORT 10013
#define APP_SERVER_MAX_CONNECTIONS 1
#define APP_SERVER_TIMEOUT 15000
#define JACK_TLS	0
#define APP_SERVER_CERT "certs/server_cert.pem"
#define APP_SERVER_KEY "certs/server_key.pem"
#define APP_CA_CERT "certs/ca_cert.pem"

//Client's PSK identity
#define APP_CLIENT1_PSK_IDENTITY "Client1"
#define APP_CLIENT2_PSK_IDENTITY "Client2"

//Client's PSK
static const uint8_t client1Psk[] = {
   0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
   0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

static const uint8_t client2Psk[] = {
   0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
   0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};

//List of preferred ciphersuites
static const uint16_t cipherSuites[] =
{
   TLS_CHACHA20_POLY1305_SHA256,
   TLS_AES_128_GCM_SHA256
};

//Global variables
static uint_t hitCounter = 0;

static OsSemaphore connectionSemaphore;
static TlsTicketContext tlsTicketContext;

extern YarrowContext yarrowContext;

//Forward declaration of functions
void jackServerTask(void *param);
void jackClientTask(void *param);

static error_t tlsServerPskCallback(TlsContext *context, const uint8_t *pskIdentity,
   size_t pskIdentityLen);

static size_t dumpArray(char_t *buffer, const uint8_t *data, size_t length);



uint16_t accelDataForJack;
uint32_t accelConfidenceForJack = 0;
uint32_t accelConfidenceForLogging = 0;

union U_Jack_RadarMsg_cartesian	radarDataForJack_cartesian[10];
struct STR_Jack_RadarMsg_polar	radarDataForJack_polar[10];
uint32_t radarConfidenceForJack = 0;
uint32_t radarConfidenceForLogging = 0;
uint16_t pirDataForJack[2];
uint32_t pirConfidenceForJack = 0;
uint32_t pirConfidenceForLogging = 0;

static TaskHandle_t xTaskToNotify_Jack = NULL;

TimerHandle_t SwTimerHandle_JackTCPIP = NULL;
SemaphoreHandle_t xJackMutex;
SemaphoreHandle_t xJackConnectionMutex;
SemaphoreHandle_t xJackStreamStartStopMutex;

static uint8_t RXBuffer[JACK_BUFFER_SIZE];

uint64_t jackMessageSequence = 0;
uint64_t jackMessageSequenceAppNull = 0;
uint64_t jackMessageSequenceAppAlarm= 0;
uint64_t jackMessageSequenceAppPlot= 0;

uint32_t active_streams = 0;

int jackRXCount = 0;

//static error_t err = NO_ERROR;

//static struct netconn *jackConn;
//int listen_JackSock, jackSocketHandle;
Socket *listen_JackSock;
Socket *jackSocketHandle;
//static TaskHandle_t xJackServerTaskHandle = NULL;
//static TaskHandle_t xJackStreamerTaskHandle = NULL;

struct STR_Jack_MSG_1_2 jackMessage;


uint8_t diagnostic_change_flag = 0;

// TODO: put these flags into spots that make sense
bool tamper_flag = 0;
bool internalErr_flag = 0;

bool reserved1_flag = 0;
bool reserved2_flag = 0;
bool reserved3_flag = 0;
bool reserved4_flag = 0;
bool accelOffline_flag = 0;
bool lPIROffline_flag = 0;
bool rPIROffline_flag = 0;
bool radarOffline_flag = 0;
bool hfMEMSOffline_flag = 0;
bool cameraOffline_flag = 0;
uint32_t diagnostic_flags = 0;
uint32_t diagnostic_flagsSent = 0;
//bool inputPower_flag = 0;
//uint32_t power_flags = 0;

bool sideA_networkFail_flag = 0;
bool sideB_networkFail_flag = 0;
uint32_t networkFail_flags = 0;

//extern struct str_powerBuffers powerBuffers;
extern enum enumPowerState powerState;


//uint32_t jack_construct_header(uint32_t msgType){

//}
static uint8_t newReset = 1; // used in device ID message to indicate new/unreported reset

extern int writeMsg_withTimeout_socket(Socket *sock, uint8_t *dataptr, size_t size);

void ones_complement(uint8_t* inBuff, uint8_t* outBuff, uint32_t length){
	for (int i = 0; i < length; i++){
		outBuff[i] = ~(inBuff[i]);
	}
}
uint64_t app_jack_get_timestamp(uint32_t message_type){
	uint64_t timestamp = 0;
	// for now the timestamp is just a sequence number specific to each type of message
	switch(message_type){
		case JACK_MSG_TYPE_APP_NULL:
			timestamp = jackMessageSequenceAppNull++; // set timestamp to current sequence number then increment
			break;
		case JACK_MSG_TYPE_APP_CONF: break; // not yet implemented
		case JACK_MSG_TYPE_APP_UCM_PLOT:
			timestamp = jackMessageSequenceAppPlot++;
			break;
		case JACK_MSG_TYPE_APP_AV: break; // not yet implemented
		case JACK_MSG_TYPE_APP_GSD: break; // not yet implemented
		case JACK_MSG_TYPE_APP_AUDIO: break; // not yet implemented
		case JACK_MSG_TYPE_APP_ALARM:
			timestamp = jackMessageSequenceAppAlarm++; // set timestamp to current sequence number then increment
			break;
		default:
			PRINTF("Jack Server Task: request type not recognized in timestamp function (rcvd TYPE=%u)\r\n", message_type);
	}
	return timestamp;
}

// placeholder function. gets information needed for sensor alarm message alarm bitfield and populates it in that form
uint32_t app_jack_get_alarm_status(void){
	uint32_t alarm_result_bitfield = 0;
	// check accelerometer
	alarm_result_bitfield |= (accel_sensor_status.alarm ? JACK_APP_ALARM_ACCEL_MASK: 0);
	// check radar
	alarm_result_bitfield |= (microwave_sensor_status.alarm ? JACK_APP_ALARM_RADAR_MASK: 0);
	// check LPIR
	alarm_result_bitfield |= (pir_sensor_status[0].alarm ? JACK_APP_ALARM_LPIR_MASK: 0);
	// check RPIR
	alarm_result_bitfield |= (pir_sensor_status[1].alarm ? JACK_APP_ALARM_RPIR_MASK: 0);
	// check HFMEMS -- not yet implemented

	// check camera -- not yet implemented

	// check switch 1 tamper -- not yet implemented

	// check switch 2 tamper -- not yet implemented

	// check combination alarm
	// not actually real combination alarm. just set it to the AND of the current alarm set
	if(alarm_result_bitfield & (JACK_APP_ALARM_ACCEL_MASK | JACK_APP_ALARM_RADAR_MASK | JACK_APP_ALARM_LPIR_MASK | JACK_APP_ALARM_RPIR_MASK)){
		alarm_result_bitfield |= JACK_APP_ALARM_COMBINATION_MASK;
	}

	return alarm_result_bitfield;
}


// separate function to act as an interface, allowing us to change the implementation of how we send messages (i.e. switching to TLS)
error_t jack_send(uint8_t* message, uint32_t length){
	error_t err = NO_ERROR;
	//err |= netconn_write(jackConn, (uint8_t *)&(message), length, NETCONN_NOCOPY);
//	xSemaphoreTake( xJackConnectionMutex, ( TickType_t ) portMAX_DELAY );
	err |= writeMsg_withTimeout_socket(jackSocketHandle, message, length);
//	xSemaphoreGive( xJackConnectionMutex);
	return err;
}

void construct_jack_header(jack_header_t *header, uint32_t length, uint32_t type, uint32_t version){
	header->STX = STX_VAL;
	header->LEN = length;
	header->TYPE = type;
	header->VER = version;
}

void construct_jack_timeCritical_subheader(jack_timeCritical_subheader_t *subheader, uint32_t nodeNum, uint64_t timestamp){
	subheader->NODE = nodeNum;
	subheader->TIME = timestamp;

}


// ----- Senders for Stream-type Messages -----
error_t jack_send_ucm_plot(uint64_t timestamp){
	jack_msg_app_ucm_plot_t message;
	error_t sendErr = NO_ERROR;
	uint32_t length;

	// ---- collect plot data ----
	message.accelMagnitude = getAccelMagnitude();
	message.leftPIRMagnitude = getLeftPIRMagnitude();
	message.rightPIRMagnitude = getRightPIRMagnitude();
	message.HFMEMSMagnitude = getHFMEMSMagnitude();
	message.cameraMagnitude = getCameraMagnitude();
	message.radarMagnitude = getRadarMagnitude();
	message.jpegData = NULL;

#if(JACK_SEND_IMAGES)
	message.jpegData = getVideoElementPtr(&message.jpegSize);
#endif
	getRadarXY((float *)message.radarXYMagnitude);




	length = (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)) - sizeof(message.jpegData);
	if(message.jpegData != NULL){
		length += message.jpegSize;
	}

	// ---- construct plot message header ----
	construct_jack_header(&message.header, length, JACK_MSG_TYPE_APP_UCM_PLOT, JACK_MSG_VER_APP_UCM_PLOT);
	construct_jack_timeCritical_subheader(&message.subheader, node_num, timestamp); // node_num is the global node number assigned on startup

	// ---- send ----
	sendErr |= jack_send((uint8_t*) &(message), sizeof(message) - sizeof(message.jpegData));		// note here that we're subtracting the size of the jpeg pointer...not the jpeg itself.
																									// this is because we don't send that pointer... we send the jpg itself... if it exists.
	if(message.jpegData != NULL){
		sendErr |= jack_send((uint8_t*) (message.jpegData), message.jpegSize);						// we're here if jpg exists.  so send it.
		free(message.jpegData);
	}
	return sendErr;
}


void refreshDiagnosticFlags(void){
	// populate diagnostic flags
	// this method sets or clears bits in the flags var w/o branching
	diagnostic_flags ^= (-tamper_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_TAMPER_MASK; // not done
	diagnostic_flags ^= (-internalErr_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_INTERNALERR_MASK; // not done
	diagnostic_flags ^= (-sync_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_1588SYNC_MASK;
	diagnostic_flags ^= (-reserved1_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RESERVED1_MASK;
	diagnostic_flags ^= (-reserved2_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RESERVED2_MASK;
	diagnostic_flags ^= (-defaultConfigFlag ^ diagnostic_flags) & JACK_DIAG_STATUS_DEFAULTCFG_MASK; // not done
	diagnostic_flags ^= (-reserved3_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RESERVED3_MASK;
	diagnostic_flags ^= (-reserved4_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RESERVED4_MASK;

	diagnostic_flags ^= (-getAccelError() ^ diagnostic_flags) & JACK_DIAG_STATUS_ACCELFAULT_MASK;
	diagnostic_flags ^= (-accelOffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_ACCELOFFLINE_MASK;

	diagnostic_flags ^= (-getLeftPIRError() ^ diagnostic_flags) & JACK_DIAG_STATUS_LPIRFAULT_MASK;
	diagnostic_flags ^= (-lPIROffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_LPIROFFLINE_MASK;
	diagnostic_flags ^= (-getRightPIRError() ^ diagnostic_flags) & JACK_DIAG_STATUS_RPIRFAULT_MASK;
	diagnostic_flags ^= (-rPIROffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RPIROFFLINE_MASK;

	diagnostic_flags ^= (-getRadarError() ^ diagnostic_flags) & JACK_DIAG_STATUS_RADARFAULT_MASK;
	diagnostic_flags ^= (-radarOffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RADAROFFLINE_MASK;

	diagnostic_flags ^= (-getHFMEMSError() ^ diagnostic_flags) & JACK_DIAG_STATUS_HFMEMSFAULT_MASK;
	diagnostic_flags ^= (-hfMEMSOffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_HFMEMSOFFLINE_MASK;

	diagnostic_flags ^= (-getCameraError() ^ diagnostic_flags) & JACK_DIAG_STATUS_CAMFAULT_MASK;
	diagnostic_flags ^= (-cameraOffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_CAMOFFLINE_MASK;

	networkFail_flags ^= (-(interface->switchDriver->getLinkState(interface, KSZ9563_PORT1)) ^ networkFail_flags) & JACK_DIAG_NETWORK_SIDEAFAIL_MASK; // side A
	networkFail_flags ^= (-(interface->switchDriver->getLinkState(interface, KSZ9563_PORT2)) ^ networkFail_flags) & JACK_DIAG_NETWORK_SIDEBFAIL_MASK; // side B

}



error_t jack_send_ucm_diagnostic(uint32_t subtype){
	jack_msg_diag_t message;
	error_t sendErr = NO_ERROR;
	switch(subtype){
		case JACK_DIAG_SUBTYPE_1:

			refreshDiagnosticFlags();

			message.SUBTYPE = subtype;
			message.diagnosticStatusFlags = diagnostic_flags; // diagnostic flags are separate so that we have persistence across messages as needed
//JACK_DIAG_STATUS_TAMPER_MASK
//|JACK_DIAG_STATUS_INTERNALERR_MASK
//|JACK_DIAG_STATUS_1588SYNC_MASK
////|JACK_DIAG_STATUS_RESERVED1_MASK
////|JACK_DIAG_STATUS_RESERVED2_MASK
//|JACK_DIAG_STATUS_DEFAULTCFG_MASK
////|JACK_DIAG_STATUS_RESERVED3_MASK
////|JACK_DIAG_STATUS_RESERVED4_MASK
//|JACK_DIAG_STATUS_ACCELFAULT_MASK
////|JACK_DIAG_STATUS_ACCELOFFLINE_MASK
//|JACK_DIAG_STATUS_LPIRFAULT_MASK
////|JACK_DIAG_STATUS_LPIROFFLINE_MASK
//|JACK_DIAG_STATUS_RPIRFAULT_MASK
////|JACK_DIAG_STATUS_RPIROFFLINE_MASK
//|JACK_DIAG_STATUS_RADARFAULT_MASK
////|JACK_DIAG_STATUS_RADAROFFLINE_MASK
//|JACK_DIAG_STATUS_HFMEMSFAULT_MASK
////|JACK_DIAG_STATUS_HFMEMSOFFLINE_MASK
//|JACK_DIAG_STATUS_CAMFAULT_MASK
////|JACK_DIAG_STATUS_CAMOFFLINE_MASK
//;
			diagnostic_flagsSent = diagnostic_flags;
			// this will likely have to be changed as we get more bit definitions than just input fault
			message.voltageRailFaults = (uint32_t) ((powerState == powerState_lowVin)||(powerState == powerState_fault)) ?
					JACK_DIAG_POWER_INPUTFAULT_MASK : 0;
			message.networkFaults = networkFail_flags;
			break;
		default:
			PRINTF("Jack Server Task: Diagnostic Send Subtype Invalid (rcvd %x)\r\n",
							subtype);
			break;
		}

	// null message currently only requires ones complement of request's data field to be sent back
	//ones_complement((uint8_t*)&request.data, (uint8_t*)&message.data, JACK_MSG_SIZE_NULL);

	// ---- construct reply header ----
	construct_jack_header(&message.header, (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)), JACK_MSG_TYPE_DIAG, JACK_MSG_VER_DIAG);

	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));
	return sendErr;
}

// ----- Application Request Handlers -----
error_t handle_app_null_req(jack_header_t *reqHeader, uint8_t* rcvBuf, uint64_t timestamp){
	jack_msg_app_null_req_t request;
	jack_msg_app_null_t message;
	error_t sendErr = NO_ERROR;

	// ---- organize request ----
	request.header = *reqHeader;
	memcpy(&request.data, &rcvBuf[JACK_HEADER_SIZE], JACK_MSG_SIZE_APP_NULL_REQ);

	// ---- process it ----
	// TODO add error checking for request
	// null message currently only requires ones complement of request's data field to be sent back
	ones_complement((uint8_t*)&request.data, (uint8_t*)&message.data, sizeof(message.data));

	// ---- construct reply header ----
	construct_jack_header(&message.header, (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)), JACK_MSG_TYPE_APP_NULL, JACK_MSG_VER_APP_NULL);
	construct_jack_timeCritical_subheader(&message.subheader, node_num, timestamp); // node_num is the global node number assigned on startup

	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &(message), (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)));
	return sendErr;
}

error_t handle_app_plot_req(jack_header_t *reqHeader, uint8_t* rcvBuf, uint64_t timestamp){
	jack_msg_app_plot_req_t request;
	//jack_msg_app_plot_t message;
	error_t sendErr = NO_ERROR;
	uint8_t notify_streamer = 0;

	// ---- organize request ----
	request.header = *reqHeader;
	memcpy(&request.streamingControl, &rcvBuf[JACK_HEADER_SIZE], JACK_MSG_SIZE_APP_UCM_PLOT_REQ);

	// ---- process it ----
	// TODO add error checking for request
	if(request.streamingControl == JACK_APP_PLOT_STREAM_STOP){ // only works if stop stream is 0x00000000
		active_streams &= ~JACK_STREAM_PLOT_MASK; // disable plot stream only
		if (!active_streams){
			// if that was the only active stream then block the streamer task
		}
	}
	// streaming control can be ORd like 0x3 for start streaming plot but also send a plot message right now
	if(request.streamingControl & JACK_APP_PLOT_STREAM_START){
		if(active_streams == 0){
			// if there are currently no active streams, be sure to notify the streamer task to unblock it
			notify_streamer = 1;
		}
		active_streams |= JACK_STREAM_PLOT_MASK; // enable stream

		if(notify_streamer){
			// unblock streamer task
		}
	}
	if(request.streamingControl & JACK_APP_PLOT_STREAM_SEND1){
		sendErr |= jack_send_ucm_plot(timestamp); // just send 1
	}

	// ---- construct reply header ----
	//construct_jack_header(&message.header, JACK_MSG_SIZE_APP_PLOT, JACK_MSG_TYPE_APP_PLOT, JACK_MSG_VER_APP_PLOT);
	//construct_jack_timeCritical_subheader(&message.subheader, node_num, timestamp); // node_num is the global node number assigned on startup

	// ---- reply ----
	//sendErr |= jack_send((uint8_t*) &(message), JACK_HEADER_SIZE+JACK_TIMECRITICAL_SUBHEADER_SIZE+JACK_MSG_SIZE_APP_PLOT);
	return sendErr;
}

error_t handle_app_alarm_req(jack_header_t *reqHeader, uint8_t* rcvBuf, uint64_t timestamp){
	jack_msg_app_alarm_req_t request;
	jack_msg_app_alarm_t message;
	error_t sendErr = NO_ERROR;

	// ---- organize request ----
	request.header = *reqHeader;
	memcpy(&request.requestSubtype, &rcvBuf[JACK_HEADER_SIZE], JACK_MSG_SIZE_APP_ALARM_REQ);

	// ---- process it ----
	// TODO add error checking for request
	switch(request.requestSubtype){
		case JACK_APP_ALARM_SUBTYPE_1:
			// populate
			message.SUBTYPE = request.requestSubtype;
			message.alarm_bitfield = app_jack_get_alarm_status();
			break;
		default:
			PRINTF("Jack Server Task: Alarm Request Subtype Invalid (rcvd %x)\r\n",
							request.requestSubtype);
			break;
		}

	// ---- construct reply header ----
	construct_jack_header(&message.header, (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)), JACK_MSG_TYPE_APP_ALARM, JACK_MSG_VER_APP_ALARM);
	construct_jack_timeCritical_subheader(&message.subheader, node_num, timestamp); // node_num is the global node number assigned on startup

	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));
	return sendErr;
}

// ----- Common/Housekeeping Request Handlers -----
error_t handle_null_req(jack_header_t *reqHeader, uint8_t* rcvBuf){
//	jack_msg_null_req_t request;
	jack_msg_null_t message;
	error_t sendErr = NO_ERROR;

	// ---- organize request ----
//	request.header = *reqHeader;
//	memcpy(&request.data, &rcvBuf[JACK_HEADER_SIZE], JACK_MSG_SIZE_NULL_REQ);

	// ---- process it ----
	// TODO add error checking for request
	// null message currently only requires ones complement of request's data field to be sent back
//	ones_complement((uint8_t*)&request.data, (uint8_t*)&message.data, JACK_MSG_SIZE_NULL);

	// ---- construct reply header ----
	construct_jack_header(&message.header, (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)), JACK_MSG_TYPE_NULL, JACK_MSG_VER_NULL);

	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));
	return sendErr;
}

error_t handle_deviceID_req(jack_header_t *reqHeader, uint8_t* rcvBuf){
//	jack_msg_deviceID_req_t request;
	jack_msg_deviceID_t message;
	error_t sendErr = NO_ERROR;

	// ---- organize request ----
//	request.header = *reqHeader;


	// ---- process it ----
	message.deviceType = 	DEVICE_TYPE;
	message.fwVersion = 	FIRMWARE_VERSION;
	message.reserved1 = 	0;
	message.serialNumber = 	SERIAL_NUMBER_BASE + node_num;
	message.reserved2 = 	0;
	message.hwVersion = 	HARDWARE_VERSION;
	message.reserved3 = 	0;
	message.resetCause = 	last_reset_cause;
	if (newReset){
		// this is a new/unreported reset
		message.resetCause |=  JACK_DEVICEID_NEWRESET_MASK;
		newReset = 0;
	}


	// ---- construct reply header ----
	construct_jack_header(&message.header, (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)), JACK_MSG_TYPE_DEVICEID, JACK_MSG_VER_DEVICEID);

	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));
	return sendErr;
}

error_t handle_reboot_req(jack_header_t *reqHeader, uint8_t* rcvBuf){
	jack_msg_reboot_req_t request;
	jack_msg_reboot_t message;
	error_t sendErr = NO_ERROR;

	// ---- organize request ----
	request.header = *reqHeader;
	memcpy(&request.resetRequest, &rcvBuf[JACK_HEADER_SIZE], JACK_MSG_SIZE_REBOOT_REQ);

	// ---- process it ----

	// check that message data is two u32s of 0xFF00FF00 and then

	if(request.resetRequest[0] == JACK_REBOOT_RESET_VAL &&
		request.resetRequest[1] == JACK_REBOOT_RESET_VAL){


		// ---- construct reply header ----
		construct_jack_header(&message.header, (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)), JACK_MSG_TYPE_REBOOT, JACK_MSG_VER_REBOOT);

		// ---- reply ----
		sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));

		// ---- take action ----
		vTaskDelay(100); // bandaid solution for getting message out before reset. TODO make it deterministic
		NVIC_SystemReset();	// system reset.
		resetBoard();
	}
	else{
		PRINTF("Jack Server Task: Reset Request Field Invalid (expected %x %x rcvd %x %x)\r\n",JACK_REBOOT_RESET_VAL, JACK_REBOOT_RESET_VAL,
				request.resetRequest[0], request.resetRequest[1]);
	}
	return sendErr;
}
void erase64KBlock(uint32_t addr);
void readPage(uint32_t addr, uint8_t *data);
void readPageXIP(uint32_t addr, uint8_t *data);
extern uint32_t makeBootloadHappen;

__NOINIT(BOARD_SDRAM) uint8_t FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START) + FLASH_PAGE_SIZE];
error_t handle_FWupdate_req(jack_header_t *reqHeader, uint8_t* rcvBuf){
	jack_msg_FWupdate_req_t *request = (jack_msg_FWupdate_req_t *)rcvBuf;
	jack_msg_FWupdate_t message;
	uint32_t replySize = 0;
	uint32_t index, pageIndex, imageSize;
	error_t sendErr = NO_ERROR;
	uint8_t reg;
	uint8_t flashPage[FLASH_PAGE_SIZE];

	uint16_t packetCRC = CRC_INITIAL;

	// ram functions..
//	void (*erase64KBlockFP)(uint32_t addr);
//	void (*eraseSectorFP)(uint32_t addr);
//	void (*SWSPI_INITfp)(uint32_t, uint32_t);
//	void (*readPageFP)(uint32_t addr, uint8_t *data);
//	void (*writePageFP)(uint32_t addr, uint8_t *data);

//	erase64KBlockFP = &erase64KBlock;
//	eraseSectorFP = &eraseSector;
//	SWSPI_INITfp = &SWSPI_INIT;
//	readPageFP = &readPage;
//	writePageFP = &writePage;



	// ---- organize request ----
	switch(request->SUBTYPE){
	case 1:

		request->subtype1.address &= 0x0FFFFFFF;
		if((request->subtype1.address > (FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START - 512) )){
			return 0;
		}

		memcpy(&(FW_SwapSpace[request->subtype1.address]), &(request->subtype1.data[0]), 512);

		message.SUBTYPE = 1;
		replySize += sizeof(message.SUBTYPE);
		message.subtype1.address = request->subtype1.address | 0x30000000;
		replySize += sizeof(message.subtype1.address);
		break;

	case 2:
			// Subtype 2 is UCM telling us the Checksum at the end.

		imageSize = request->subtype2.imageSize & 0x0FFFFFFF;
		packetCRC = CRC_INITIAL;
		packetCRC = CRCCalcRange(&(FW_SwapSpace[0]), imageSize);

		PRINTF("FW Write Calc CRC: %x\r\n", packetCRC);
		PRINTF("FW Write UCM CRC: %x\r\n", request->subtype2.crcChecksum);

		if((packetCRC == request->subtype2.crcChecksum))  {

				memset(&(FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START)]), 0, FLASH_PAGE_SIZE);
				((uint32_t *)(&(FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START)])))[0] = FLASH_CTRL__PartialEntityCode;
				((uint32_t *)(&(FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START)])))[1] = request->subtype2.imageSize;
				((uint32_t *)(&(FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START)])))[2] = request->subtype2.crcChecksum;

				((uint32_t *)(&(FW_SwapSpace[(FLASH_ADDR__IMAGE_1_END - FLASH_ADDR__IMAGE_1_START)])))[3] = packetCRC;		// image only CRC (no bootload section).


				makeBootloadHappen = 5;	// units here are 'heartbeat blinks'... i.e. about 0.5s per count.
				PRINTF("CRC Match... bootloading");


		}else{
			PRINTF("CRC MIS-Match");
		}

		message.SUBTYPE = 2;
		replySize += sizeof(message.SUBTYPE);
		message.subtype2.crcChecksum = packetCRC;
		replySize += sizeof(message.subtype2.crcChecksum);
	break;
	default:
		return 0;
	}

	// ---- construct reply header ----
	replySize += sizeof(message.header.TYPE) + sizeof(message.header.VER);
	construct_jack_header(&message.header, replySize, JACK_MSG_TYPE_FWUPDATE, JACK_MSG_VER_FWUPDATE);

	replySize += sizeof(message.header.LEN) + sizeof(message.header.STX);
	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &message, replySize);
	return sendErr;
}

extern void saveNewSettingsToFlash(config_param_t *newSettings);




error_t handle_config_req(jack_header_t *reqHeader, uint8_t* rcvBuf){
	jack_msg_config_req_t *requestPtr;
	jack_msg_config_t message;
	error_t sendErr = NO_ERROR;


//	config_param_t newConfig;

	// ---- organize request ----
	requestPtr = (jack_msg_config_req_t *)rcvBuf;

	// ---- process it ----

	if (requestPtr->setConfig == 1){

		sanitizeConfig(&requestPtr->configParamaters, &configParam, 6);			// sending security level 6 every time right now.

		saveNewSettingsToFlash(&requestPtr->configParamaters);
		memcpy(&configParam, &requestPtr->configParamaters, sizeof(configParam));	// make settings live.  there are some functions that make a copy though and those will need to be reset.
		setLiveSettingsVideo(0, 0, 0); // disable use of live settings so new config will be used instead
//		loadFlashSettings();		// doing this here seriously breaks stuff.  T.S 2023 08 28

		return NO_ERROR;
	}
	else if (requestPtr->setConfig == 2){
		// live settings change, don't save to flash
		sanitizeConfig(&requestPtr->configParamaters, &configParam, 6);			// sending security level 6 every time right now.

		memcpy(&configParam, &requestPtr->configParamaters, sizeof(configParam));

		return NO_ERROR;
	}
	else if (requestPtr->setConfig == 3){
		// ignore included config, reload config from flash, useful with live settings
		__disable_irq();
		loadFlashSettings();
		__enable_irq();
		return NO_ERROR;
	}
	else if(requestPtr->setConfig == 0){
		// reply with current config
		memcpy(&message.configParamaters, &configParam, sizeof(configParam));

		// ---- construct reply header ----
		construct_jack_header(&message.header, sizeof(message) - 8, JACK_MSG_TYPE_CONFIG, JACK_MSG_VER_CONFIG);

		// ---- reply ----
		sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));

		return sendErr;
	}else{
		PRINTF("Jack Server Task: Config Request Subtype Invalid\r\n");
		return ERROR_INVALID_PARAMETER;
	}
}

error_t handle_deviceHistory_req(jack_header_t *reqHeader, uint8_t* rcvBuf){
	jack_msg_devicehistory_req_t *requestPtr;
	jack_msg_devicehistory_t message;
	error_t sendErr = NO_ERROR;

	// ---- organize request ----
	requestPtr = (jack_msg_devicehistory_req_t *)rcvBuf;

	// ---- process it ----
	// TODO add error checking for request
	if (requestPtr->historyControl != 0){
		// reset history based on provided values
		if (requestPtr->historyControl & JACK_HISTORY_CONTROL_RESETPOWERFLTS){
			sys_stats.pwrstat.fault6V5_count = 0;
			sys_stats.pwrstat.fault3V3_count = 0;
			sys_stats.pwrstat.faultSideAPower_count = 0;
			sys_stats.pwrstat.faultSideBPower_count = 0;
		}
		if (requestPtr->historyControl & JACK_HISTORY_CONTROL_RESETNETWORKFLTS){
			sys_stats.nwkstat.faultSideANetwork_count = 0;
			sys_stats.nwkstat.faultSideBNetwork_count = 0;
		}
		if (requestPtr->historyControl & JACK_HISTORY_CONTROL_RESETSENSORFLTS){
			sys_stats.sensorstat.faultAccel_count = 0;
			sys_stats.sensorstat.faultLPIR_count = 0;
			sys_stats.sensorstat.faultRPIR_count = 0;
			sys_stats.sensorstat.faultRadar_count = 0;
			sys_stats.sensorstat.faultHFMEMS_count = 0;
			sys_stats.sensorstat.faultCamera_count = 0;
		}



//		saveStats(&sys_stats);
	}

	message.faults6v5 = sys_stats.pwrstat.fault6V5_count;
	message.faults3v3 = sys_stats.pwrstat.fault3V3_count;
	message.faultsSideAPower = sys_stats.pwrstat.faultSideAPower_count;
	message.faultsSideBPower = sys_stats.pwrstat.faultSideBPower_count;
	message.faultsSideANetwork = sys_stats.nwkstat.faultSideANetwork_count;
	message.faultsSideBNetwork = sys_stats.nwkstat.faultSideBNetwork_count;
	message.faultsAccel = sys_stats.sensorstat.faultAccel_count;
	message.faultsLPIR = sys_stats.sensorstat.faultLPIR_count;
	message.faultsRPIR = sys_stats.sensorstat.faultRPIR_count;
	message.faultsRadar = sys_stats.sensorstat.faultRadar_count;
	message.faultsHFMEMS = sys_stats.sensorstat.faultHFMEMS_count;
	message.faultsCamera = sys_stats.sensorstat.faultCamera_count;

//	message.faults6v5 = 1;
//	message.faults3v3 = 2;
//	message.faultsSideAPower = 3;
//	message.faultsSideBPower = 4;
//	message.faultsSideANetwork = 5;
//	message.faultsSideBNetwork = 6;
//	message.faultsAccel = 7;
//	message.faultsLPIR = 8;
//	message.faultsRPIR = 9;
//	message.faultsRadar = 10;
//	message.faultsHFMEMS = 11;
//	message.faultsCamera = 12;


	// ---- construct reply header ----
	construct_jack_header(&message.header, (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)), JACK_MSG_TYPE_HISTORY, JACK_MSG_VER_HISTORY);

	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));
	return sendErr;
}
float getRoll(void);
float getPitch(void);

error_t handle_power_req(jack_header_t *reqHeader, uint8_t* rcvBuf){
	jack_msg_power_req_t request;
	jack_msg_power_t message;
	error_t sendErr = NO_ERROR;

	// ---- organize request ----
	request.header = *reqHeader;
	memcpy(&request.requestSubtype, &rcvBuf[JACK_HEADER_SIZE], JACK_MSG_SIZE_POWER_REQ);

	// ---- process it ----
	// TODO add error checking for request
	switch(request.requestSubtype){
	case JACK_POWER_SUBTYPE_1:
		// populate
		message.SUBTYPE = request.requestSubtype;
		message.inputVoltage = powerBuffers.voltage;
		message.sideAcurrent = powerBuffers.current[0];
		message.sideBcurrent = powerBuffers.current[1];
		message.powerState = (uint32_t) powerState;
		message.flags = power_flags;
		message.roll = getRoll();
		message.pitch = getPitch();
		break;
	default:
		PRINTF("Jack Server Task: Power Request Subtype Invalid (rcvd %x)\r\n",
						request.requestSubtype);
		break;
	}

	//ones_complement((uint8_t*)&request.data, (uint8_t*)&message.data, JACK_MSG_SIZE_POWER);

	// ---- construct reply header ----
	construct_jack_header(&message.header, (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)), JACK_MSG_TYPE_POWER, JACK_MSG_VER_POWER);

	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));
	return sendErr;
}

error_t handle_diagnostics_req(jack_header_t *reqHeader, uint8_t* rcvBuf){
	jack_msg_diag_req_t request;
	jack_msg_diag_t message;
	error_t sendErr = NO_ERROR;
	//uint32_t cameraErr;


	// ---- organize request ----
	request.header = *reqHeader;
	memcpy(&request.requestSubtype, &rcvBuf[JACK_HEADER_SIZE], JACK_MSG_SIZE_DIAG_REQ);

	// ---- process it ----
	// TODO add error checking for request
	switch(request.requestSubtype){
		case JACK_DIAG_SUBTYPE_1:

			// populate diagnostic flags
			// this method sets or clears bits in the flags var w/o branching
			diagnostic_flags ^= (-tamper_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_TAMPER_MASK; // not done
			diagnostic_flags ^= (-internalErr_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_INTERNALERR_MASK; // not done
			diagnostic_flags ^= (-sync_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_1588SYNC_MASK;
			diagnostic_flags ^= (-reserved1_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RESERVED1_MASK;
			diagnostic_flags ^= (-reserved2_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RESERVED2_MASK;
			diagnostic_flags ^= (-defaultConfigFlag ^ diagnostic_flags) & JACK_DIAG_STATUS_DEFAULTCFG_MASK;
			diagnostic_flags ^= (-reserved3_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RESERVED3_MASK;
			diagnostic_flags ^= (-reserved4_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RESERVED4_MASK;

			diagnostic_flags ^= (-getAccelError() ^ diagnostic_flags) & JACK_DIAG_STATUS_ACCELFAULT_MASK;
			diagnostic_flags ^= (-accelOffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_ACCELOFFLINE_MASK;

			diagnostic_flags ^= (-getLeftPIRError() ^ diagnostic_flags) & JACK_DIAG_STATUS_LPIRFAULT_MASK;
			diagnostic_flags ^= (-lPIROffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_LPIROFFLINE_MASK;
			diagnostic_flags ^= (-getRightPIRError() ^ diagnostic_flags) & JACK_DIAG_STATUS_RPIRFAULT_MASK;
			diagnostic_flags ^= (-rPIROffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RPIROFFLINE_MASK;

			diagnostic_flags ^= (-getRadarError() ^ diagnostic_flags) & JACK_DIAG_STATUS_RADARFAULT_MASK;
			diagnostic_flags ^= (-radarOffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_RADAROFFLINE_MASK;

			diagnostic_flags ^= (-getHFMEMSError() ^ diagnostic_flags) & JACK_DIAG_STATUS_HFMEMSFAULT_MASK;
			diagnostic_flags ^= (-hfMEMSOffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_HFMEMSOFFLINE_MASK;

//			diagnostic_flags ^= (-getCameraError() ^ diagnostic_flags) & JACK_DIAG_STATUS_CAMFAULT_MASK;
//			diagnostic_flags ^= (-cameraOffline_flag ^ diagnostic_flags) & JACK_DIAG_STATUS_CAMOFFLINE_MASK;

		networkFail_flags ^= (-(interface->switchDriver->getLinkState(interface, KSZ9563_PORT1)) ^ networkFail_flags) & JACK_DIAG_NETWORK_SIDEAFAIL_MASK; // side A
		networkFail_flags ^= (-(interface->switchDriver->getLinkState(interface, KSZ9563_PORT2)) ^ networkFail_flags) & JACK_DIAG_NETWORK_SIDEBFAIL_MASK; // side B

			message.SUBTYPE = request.requestSubtype;
			message.diagnosticStatusFlags = diagnostic_flags;
//JACK_DIAG_STATUS_TAMPER_MASK
//|JACK_DIAG_STATUS_INTERNALERR_MASK
//|JACK_DIAG_STATUS_1588SYNC_MASK
////|JACK_DIAG_STATUS_RESERVED1_MASK
////|JACK_DIAG_STATUS_RESERVED2_MASK
//|JACK_DIAG_STATUS_DEFAULTCFG_MASK
////|JACK_DIAG_STATUS_RESERVED3_MASK
////|JACK_DIAG_STATUS_RESERVED4_MASK
//|JACK_DIAG_STATUS_ACCELFAULT_MASK
////|JACK_DIAG_STATUS_ACCELOFFLINE_MASK
//|JACK_DIAG_STATUS_LPIRFAULT_MASK
////|JACK_DIAG_STATUS_LPIROFFLINE_MASK
//|JACK_DIAG_STATUS_RPIRFAULT_MASK
////|JACK_DIAG_STATUS_RPIROFFLINE_MASK
//|JACK_DIAG_STATUS_RADARFAULT_MASK
////|JACK_DIAG_STATUS_RADAROFFLINE_MASK
//|JACK_DIAG_STATUS_HFMEMSFAULT_MASK
////|JACK_DIAG_STATUS_HFMEMSOFFLINE_MASK
//|JACK_DIAG_STATUS_CAMFAULT_MASK
////|JACK_DIAG_STATUS_CAMOFFLINE_MASK

					//;//diagnostic_flags; // diagnostic flags are separate so that we have persistence across messages as needed
			// this will likely have to be changed as we get more bit definitions than just input fault
			message.voltageRailFaults = (uint32_t) ((powerState == powerState_lowVin)||(powerState == powerState_fault)) ?
					JACK_DIAG_POWER_INPUTFAULT_MASK : 0;
			message.networkFaults = networkFail_flags;
			break;
		default:
			PRINTF("Jack Server Task: Diagnostic Request Subtype Invalid (rcvd %x)\r\n",
							request.requestSubtype);
			break;
		}

	// null message currently only requires ones complement of request's data field to be sent back
	//ones_complement((uint8_t*)&request.data, (uint8_t*)&message.data, JACK_MSG_SIZE_NULL);

	// ---- construct reply header ----
	construct_jack_header(&message.header, (sizeof(message) - sizeof(message.header.STX) - sizeof(message.header.LEN)), JACK_MSG_TYPE_DIAG, JACK_MSG_VER_DIAG);

	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));
	return sendErr;
}

error_t handle_livesettings_req(jack_header_t *reqHeader, uint8_t* rcvBuf){
	jack_msg_livesettings_req_t *requestPtr;
	jack_msg_livesettings_t message;
	error_t sendErr = NO_ERROR;


	// ---- organize request ----
	requestPtr = (jack_msg_livesettings_req_t *)rcvBuf;

	// ---- process it ----
	setLiveSettingsVideo(requestPtr->videoDecorations, requestPtr->imageProcessingPreviewSelect, 1); // enable live settings use

	// reply with the rcvd live settings

	message.videoDecorations = requestPtr->videoDecorations;
	message.imageProcessingPreviewSelect = requestPtr->imageProcessingPreviewSelect;

	// ---- construct reply header ----
	construct_jack_header(&message.header, sizeof(message) - 8, JACK_MSG_TYPE_LIVESETTINGS, JACK_MSG_VER_LIVESETTINGS);

	// ---- reply ----
	sendErr |= jack_send((uint8_t*) &(message), (sizeof(message)));

	return sendErr;

}

error_t process_message(uint8_t* rcvBuf){
	jack_header_t rcvHeader;
	error_t sendErr = NO_ERROR;
	uint64_t timestamp;

	memcpy(&rcvHeader, rcvBuf, sizeof(rcvHeader));				// TODO: I'm not actually convinced we need a copy.
	if (rcvHeader.STX != STX_VAL){ // check STX values match
		PRINTF("Jack Server Task: STX mismatch (rcvd STX=%u)\r\n", rcvHeader.STX);
		return -17; // TODO handle STX checking
	}
//	memcpy(&rcvHeader.LEN, &((jack_header_t *)rcvBuf)->LEN, sizeof(rcvHeader.LEN));
//	memcpy(&rcvHeader.TYPE, &((jack_header_t *)rcvBuf)->TYPE, sizeof(rcvHeader.TYPE));
//	memcpy(&rcvHeader.VER, &((jack_header_t *)rcvBuf)->VER, sizeof(rcvHeader.VER));
	if (rcvHeader.TYPE & JACK_MSG_REQUEST_MASK){ // received message is a request
		// send request to specific handler based on type requested
		switch (((~JACK_MSG_REQUEST_MASK) & rcvHeader.TYPE)){
			// Time Critical / Application Messages
			case JACK_MSG_TYPE_APP_NULL:
				// placeholder function using msg specific sequence numbers.
				timestamp = app_jack_get_timestamp(JACK_MSG_TYPE_APP_NULL);
				sendErr |= handle_app_null_req(&rcvHeader, rcvBuf, timestamp);
				break;
			case JACK_MSG_TYPE_APP_CONF: break; // not yet implemented
			case JACK_MSG_TYPE_APP_UCM_PLOT:
				timestamp = app_jack_get_timestamp(JACK_MSG_TYPE_APP_UCM_PLOT);
				sendErr |= handle_app_plot_req(&rcvHeader, rcvBuf, timestamp);
				break;
			case JACK_MSG_TYPE_APP_AV: break; // not yet implemented
			case JACK_MSG_TYPE_APP_GSD: break; // not yet implemented
			case JACK_MSG_TYPE_APP_AUDIO: break; // not yet implemented
			case JACK_MSG_TYPE_APP_ALARM:
				// placeholder function using msg specific sequence numbers.
				timestamp = app_jack_get_timestamp(JACK_MSG_TYPE_APP_ALARM);
				sendErr |= handle_app_alarm_req(&rcvHeader, rcvBuf, timestamp);
				break;

			// Housekeeping / Common Messages
			case JACK_MSG_TYPE_NULL:
				sendErr |= handle_null_req(&rcvHeader, rcvBuf);
				break;
			case JACK_MSG_TYPE_DEVICEID:
				sendErr |= handle_deviceID_req(&rcvHeader, rcvBuf);
				break;
			case JACK_MSG_TYPE_REBOOT:
				sendErr |= handle_reboot_req(&rcvHeader, rcvBuf);
				break;
			case JACK_MSG_TYPE_RAWHIST: break; // not yet implemented
			case JACK_MSG_TYPE_FLASH: break; // not yet implemented
			case JACK_MSG_TYPE_HWMAGNITUDE: break; // not yet implemented
			case JACK_MSG_TYPE_DEVDSCRIP: break; // not yet implemented
			case JACK_MSG_TYPE_DEVICEIP: break; // not yet implemented
			case JACK_MSG_TYPE_CONFIG:
				sendErr |= handle_config_req(&rcvHeader, rcvBuf);
				break;
			case JACK_MSG_TYPE_HISTORY:
				 sendErr |= handle_deviceHistory_req(&rcvHeader, rcvBuf);
				 break;
			case JACK_MSG_TYPE_POWER:
				sendErr |= handle_power_req(&rcvHeader, rcvBuf);
				break;
			case JACK_MSG_TYPE_DIAG:
				 sendErr |= handle_diagnostics_req(&rcvHeader, rcvBuf);
				 break;
			case JACK_MSG_TYPE_LIVESETTINGS:
				sendErr |= handle_livesettings_req(&rcvHeader, rcvBuf);
				break;

			default: PRINTF("Jack Server Task: request type not recognized (rcvd TYPE=%u)\r\n", rcvHeader.TYPE);
		}
	}
	else {
		// received message is not a request. not yet a thing
		switch (((~JACK_MSG_REQUEST_MASK) & rcvHeader.TYPE)){

			case JACK_MSG_TYPE_CONFIG:
				sendErr |= handle_config_req(&rcvHeader, rcvBuf);
				break; // not yet implemented
			case JACK_MSG_TYPE_FWUPDATE:
				sendErr |= handle_FWupdate_req(&rcvHeader, rcvBuf);
				break;
			case JACK_MSG_TYPE_REBOOT:
				sendErr |= handle_reboot_req(&rcvHeader, rcvBuf);
				break;
			case JACK_MSG_TYPE_LIVESETTINGS:
				sendErr |= handle_livesettings_req(&rcvHeader, rcvBuf);
				break;

			default:
				PRINTF("Jack Server Task: Command type not recognized (rcvd TYPE=%u)\r\n", rcvHeader.TYPE);

		}







	}
	return sendErr;
}
#if(0)
// this task is responsible for the application data streams for jack
void jack_streamer_task(void *pvParameters){
	// jack stream task has a task delay of ~100ms for a base rate of 10Hz
	// if a faster rate is required the function needs to be reworked for a smaller delay
	uint32_t timeslice_100ms = 0;
	uint64_t timestamp;
	error_t sendErr = NO_ERROR;

	do{
		//xSemaphoreTake( xJackStreamStartStopMutex, ( TickType_t ) portMAX_DELAY ); use task notifications instead
		if(!(timeslice_100ms % 500)&& // 2Hz
				(active_streams & JACK_STREAM_PLOT_MASK)){
			timestamp = app_jack_get_timestamp(JACK_MSG_TYPE_APP_UCM_PLOT);
			sendErr |= jack_send_ucm_plot(timestamp);
		}

		timeslice_100ms += 100;
		if(timeslice_100ms >= 10000){ // arbitrary nice number. reset the counter @ 10s
			timeslice_100ms = 0;
		}

//		if (err != NO_ERROR) {
//					PRINTF("Jack Streamer Task: error on send (err=%d)\r\n", err);
//					continue;		// TODO handle this
//				}
//		else{
//				err = NO_ERROR;
//			}
		// ERR is a shared global variable that any mistakes on any port can cause this to change.  This is dumb.  We need to stop using it.

		vTaskDelay(100); // 100ms
	}while(sendErr == NO_ERROR);

	vTaskDelete(NULL);

}
#endif

// this task handles replying to jack messages
void jack_server_task(void *pvParameters){
//	struct netbuf *net_buf;
//	uint8_t buffer[1024];
	error_t local_err = NO_ERROR;
	//uint32_t recvType;
	//uint32_t STXfld, LENfld, VERfld;
	//uint32_t data[((1024-JACK_HEADER_SIZE)/4)];
	//uint32_t sendData[((1024-JACK_HEADER_SIZE)/4)];
#if(0)
	if (xTaskCreate(jack_streamer_task, "jack_streamer_task", 2048, NULL, DEFAULT_THREAD_PRIO, &xJackStreamerTaskHandle) != pdPASS)
				{
					PRINTF("Jack Streamer task creation failed!\r\n");
					while (1)
						;
				}
#endif
	do{
#if(1)
		vTaskDelay(1000);
#else
		//ulTaskNotifyTake(pdFALSE, ( TickType_t ) portMAX_DELAY); // don't clear the notification count
																// just decrement one for the one message received
		//recvType = 0;
		xSemaphoreTake( xJackConnectionMutex, ( TickType_t ) portMAX_DELAY );
		jackRXCount = recv(jackSocketHandle, RXBuffer, sizeof(RXBuffer) - 1, 0);
		xSemaphoreGive( xJackConnectionMutex);
		if(jackRXCount < 0){
			if(errno == ENOTCONN){
				local_err = -1;	// ENOTCONN means the other end has blown away the connection.  We need to reset.
				xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
					PRINTF("Jack port closed: errno %d\r\n", errno);
				xSemaphoreGive(xPrintMutex);
			}
			else if(errno != EAGAIN){
//						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
//						PRINTF("Logging Error occurred during receiving: errno %d\r\n", errno);
//						xSemaphoreGive( xPrintMutex );
			}
		} else if (jackRXCount == 0) {
				// nothing to receive.
			vTaskDelay(50);
		}
		else{
			local_err = NO_ERROR;
			local_err |= process_message(RXBuffer);
		}

		if (local_err != NO_ERROR) {
			PRINTF("Jack Server Task: error on send (err=%d)\r\n", local_err);
			continue;		// TODO handle this
		}
		else{
			local_err = NO_ERROR;
		}
#endif
	}
	while(local_err == NO_ERROR);
	vTaskDelete(NULL);
}



#if(0)
// this task handles the connection to the client and spawns a jack server task when a connection is established
void tcp_Jack_task_old(void *pvParameters)
{
	TickType_t wakeTimer = 0, wakeTimerPrev = 0;



	uint32_t i;
	int opt;
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int jackRXCount = 0;
    error_t local_err;
	uint32_t timeslice_ms = 0;
	struct sockaddr_storage dest_addr;
//	uint64_t timestamp = 0;
	struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);
    uint32_t rxBufferIndex = 0;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
	dest_addr_ip4->sin_family = AF_INET;
	dest_addr_ip4->sin_port = htons(PORT);


	//	uint32_t ulNotificationValue;

	memset(&jackMessage, 0, sizeof(jackMessage));


	LWIP_UNUSED_ARG(pvParameters);
	/* Store the handle of the calling task. */
	xTaskToNotify_Jack = xTaskGetCurrentTaskHandle();



	/* Main loop. Get sensor data and send via TCP */
	while (1)
	{


		listen_JackSock = socket(addr_family, SOCK_STREAM, IPPROTO_TCP);
		if (listen_JackSock < 0) {
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Unable to create Jack socket: errno %d\r\n", errno);
			xSemaphoreGive( xPrintMutex );
			break; //goto CLEAN_UP;
		}
		opt = 1;
		local_err = setsockopt(listen_JackSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
		if(local_err != NO_ERROR){PRINTF("JACK - socket set opt Fail\r\n");}

		// I'm not actually sure if we need linger turned on.  But it works.
		const struct linger linger = {.l_onoff = 1, .l_linger = 5};
		local_err = setsockopt(listen_JackSock, SOL_SOCKET, SO_LINGER, &linger, sizeof(linger));
		if(local_err != NO_ERROR){PRINTF("socket set opt Fail\r\n");}

		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Jack Socket created!\r\n");
		xSemaphoreGive( xPrintMutex );

		local_err = bind(listen_JackSock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if (local_err != 0) {
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Jack - Socket unable to bind: errno %d\r\n", errno);
			PRINTF("IPPROTO: %d", addr_family);
			xSemaphoreGive( xPrintMutex );
			break;  //goto CLEAN_UP;
		}

		xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
		PRINTF("Jack Socket bound, port %d\r\n", PORT);
		xSemaphoreGive( xPrintMutex );


		local_err = listen(listen_JackSock, 1);
		if (local_err != 0) {
			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Error occurred during listen - Jack Socket: errno %d\r\n", errno);
			xSemaphoreGive( xPrintMutex );
			break;//goto CLEAN_UP;
		}




		while (1) {

			xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Jack Socket listening\r\n");
			xSemaphoreGive( xPrintMutex );

	        jackSocketHandle = accept(listen_JackSock, (struct sockaddr *)&source_addr, &addr_len);
	        if (jackSocketHandle < 0) {
		    	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
				PRINTF("Unable to accept connection: - Jack Socket: errno %d\r\n", errno);
				xSemaphoreGive( xPrintMutex );
	            break;
	        }

			// Set tcp keepalive option
	        int keepAlive = 1;	// 1= enable keep alive
	        int keepIdle = 5;	// 5 = send first keepalive probe after 5 seconds of idleness;
	        int keepInterval = 3;// 3 = send subsequent keepalive probels after 3 seconds.
	        int keepCount = 3; 	// 3 = timeout after 3 failed probes.;
			local_err = setsockopt(jackSocketHandle, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
			local_err = setsockopt(jackSocketHandle, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
			local_err = setsockopt(jackSocketHandle, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
			local_err = setsockopt(jackSocketHandle, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

			// enable send timeout.
		    struct timeval sendTimeout, recvTimeout;
		    sendTimeout.tv_sec = 2;
		    sendTimeout.tv_usec = 0;		// 2s
		    recvTimeout.tv_sec = 0;
		    recvTimeout.tv_usec = 10000;	//0.01 S
			//{
		    //  long    tv_sec;         /* seconds */
		    //  long    tv_usec;        /* and microseconds */
		    //};
	        local_err = setsockopt(jackSocketHandle, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));	// enable send timeout.
	        local_err = setsockopt(jackSocketHandle, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(sendTimeout));	// enable rx timeout.

	        // Convert ip address to string
	        if (source_addr.ss_family == PF_INET) {
	            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
	        }
	        i = ((((struct sockaddr_in *)&source_addr)->sin_port) & 0x00FF) * 256;
	        i = i + ((((struct sockaddr_in *)&source_addr)->sin_port) >> 8);



	    	xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
			PRINTF("Jack Socket accepted ip address: %s : %d\r\n", addr_str, i);
			xSemaphoreGive( xPrintMutex );
			rxBufferIndex = 0;
			jackRXCount = 0;
		wdog_networkActivity |= 1;
		wakeTimer = xTaskGetTickCount();




			do {
//				vTaskDelay(75);
				// streaming messages will go here and will be organized by timeslice

				xSemaphoreTake( xJackConnectionMutex, ( TickType_t ) portMAX_DELAY );
				if((rxBufferIndex < 0) || (rxBufferIndex > 1460)){
					rxBufferIndex = 0;	// fault scenario.
					jackRXCount = 0;
				}
				if((jackRXCount < 0) || (jackRXCount > 1460)){
					rxBufferIndex = 0;	// fault scenario.
					jackRXCount = 0;
				}
				jackRXCount += recv(jackSocketHandle, &(RXBuffer[rxBufferIndex]), sizeof(RXBuffer) - 1 - rxBufferIndex, 0);
				xSemaphoreGive( xJackConnectionMutex);
				if(jackRXCount < 0){
					if(errno == ENOTCONN){
						local_err = -1;	// ENOTCONN means the other end has blown away the connection.  We need to reset.
						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
							PRINTF("Jack port closed: errno %d\r\n", errno);
						xSemaphoreGive(xPrintMutex);
					}
					else if(errno != EAGAIN){		//EAGAIN could also be 'would block'
					}
				}


				uint64_t timestamp;
				int32_t msgLength;


				while(jackRXCount > 0){

					// check for alignment.
					if(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->STX !=  STX_VAL){
						// mis-aligned.
						rxBufferIndex++;
						if(rxBufferIndex >= (sizeof(RXBuffer) -1)){
							jackRXCount = -1;	// fault condition.
						}
						jackRXCount--;
						if(jackRXCount == 0){
							rxBufferIndex = 0;
						}
						continue;
					}

					msgLength = (((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->LEN +
												sizeof(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->LEN) +
												sizeof(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->STX)  );
					// check that the Length byte is reasonable.
					if((msgLength > ((JACK_BUFFER_SIZE) * 2 / 3)) || (msgLength < sizeof(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->LEN) + sizeof(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->STX)))
					{
						// Length byte is unreasonable.  This is an error.  Treat this as mis-alignment.
						rxBufferIndex++;
						if(rxBufferIndex >= (sizeof(RXBuffer) -1)){
							jackRXCount = -1;	// fault condition.
						}
						jackRXCount--;
						if(jackRXCount == 0){
							rxBufferIndex = 0;
						}
						continue;
					}
					// check that the entire msg is in the RXBuffer.
					if(msgLength > jackRXCount)
					{
						break;	// break loop so we can receive more bytes.
					}

					local_err = process_message(&(RXBuffer[rxBufferIndex]));

					if((local_err < 0) || (msgLength > jackRXCount)){
						jackRXCount = 0;
//						local_err = -1;
						rxBufferIndex = 0;
						continue;
					}else if( msgLength > sizeof(RXBuffer) - 1 - rxBufferIndex){
						// this is an error condition.  I.e. an error in the process_message function.  we received too many bytes.
						jackRXCount = 0;
						local_err = -1;
						rxBufferIndex = 0;
						continue;
					}
					if(msgLength == jackRXCount){
						// we received all the bytes in the queue.  Reset the rxBufferIndex
						jackRXCount = jackRXCount - msgLength;
						local_err = 0;
						rxBufferIndex = 0;

					}else if(msgLength == 0){
						// we didn't receive any bytes.  Likely because the whole msg isn't in the receive buffer.
						// abort our loop and receive more bytes.
						local_err = 0;
						break;
					}else{
						// there's more bytes in the RXBuffer that need to be processed (complete additional msg or not).
						jackRXCount = jackRXCount - msgLength;
						local_err = 0;
						rxBufferIndex += msgLength;

					}
				}



				if (local_err != NO_ERROR) {
					PRINTF("Jack Server Task: error on Receive (err=%d)\r\n", local_err);
//					continue;		// TODO handle this
					break;	// handled!
				}


				if(jackRXCount == 0){
					vTaskDelay(50);	// nothing to receive last time.
				}


// streaming code //

#define TIME_BETWEEN_MESSAGES_mS	250

				wakeTimer = xTaskGetTickCount() / portTICK_PERIOD_MS;
				if(wakeTimer > wakeTimerPrev){
					timeslice_ms += wakeTimer - wakeTimerPrev;
				}else{
					timeslice_ms += (portMAX_DELAY - wakeTimerPrev) + wakeTimer;
				}
				wakeTimerPrev = wakeTimer;
				if(timeslice_ms > TIME_BETWEEN_MESSAGES_mS)
				{
					timeslice_ms = timeslice_ms - TIME_BETWEEN_MESSAGES_mS;
					 // 2Hz
					if(diagnostic_change_flag){
						local_err |= jack_send_ucm_diagnostic(JACK_DIAG_SUBTYPE_1);
						if(local_err == NO_ERROR){
							// clear the diagnostic change flag so we don't keep sending diag messages
							diagnostic_change_flag = 0;
						}
					}

					if(active_streams & JACK_STREAM_PLOT_MASK){
						timestamp = app_jack_get_timestamp(JACK_MSG_TYPE_APP_UCM_PLOT);
						local_err |= jack_send_ucm_plot(timestamp);
					}else{
						timeslice_ms = 0;
					}
				}
				if(timeslice_ms > TIME_BETWEEN_MESSAGES_mS){
					timeslice_ms = TIME_BETWEEN_MESSAGES_mS;
				}
// streaming code //

				if (local_err != NO_ERROR) {
					PRINTF("Jack Server Task: error on Receive (err=%d)\r\n", local_err);
//					continue;		// TODO handle this
					break;	// handled!
				}

				if(local_err == NO_ERROR){
					wdog_networkActivity |= 1;
				}



			}while(local_err == NO_ERROR);

	        shutdown(jackSocketHandle, SHUT_RDWR);		// Shutdown Read/Write.
	        close(jackSocketHandle);
		}

		//	CLEAN_UP:

		shutdown(listen_JackSock, SHUT_RDWR);	//TS
		close(listen_JackSock);

	}
}
#endif

/**
 * @brief // this task handles the connection to the client and spawns a jack client task when a connection is established
 * @param param[in] Not used
 **/

void jackServerTask(void *param)
{
   error_t error;
   uint_t counter;
   uint16_t clientPort;
   IpAddr clientIpAddr;
   Socket *serverSocket;
   Socket *clientSocket;
   portBASE_TYPE status;
   TaskHandle_t handle;

   //Create a semaphore to limit the number of simultaneous connections
   if(!osCreateSemaphore(&connectionSemaphore, APP_SERVER_MAX_CONNECTIONS))
   {
      //Debug message
      TRACE_ERROR("Failed to create semaphore!\r\n");
   }

#if (APP_JACK_TLS && TLS_TICKET_SUPPORT == ENABLED)
   //Initialize ticket encryption context
   error = tlsInitTicketContext(&tlsTicketContext);
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to bind socket!\r\n");
   }
#endif

   //Open a socket
   serverSocket = socketOpen(SOCKET_TYPE_STREAM, SOCKET_IP_PROTO_TCP);
   //Failed to open socket?
   if(!serverSocket)
   {
      //Debug message
      TRACE_ERROR("Cannot open socket!\r\n");
   }

   //Bind newly created socket to port 443
   error = socketBind(serverSocket, &IP_ADDR_ANY, APP_SERVER_PORT);
   //Failed to bind socket to port 443?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to bind socket!\r\n");
   }

   //Place socket in listening state
   error = socketListen(serverSocket, 1);
   //Any failure to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to enter listening state!\r\n");
   }

   //Process incoming connections to the server
   for(counter = 1; ; counter++)
   {
      //Debug message
      TRACE_INFO("\r\n\r\n");
      TRACE_INFO("Waiting for an incoming Jack connection...\r\n\r\n");

      //Limit the number of simultaneous connections to the Jack server
      osWaitForSemaphore(&connectionSemaphore, INFINITE_DELAY);

      //Accept an incoming connection
      clientSocket = socketAccept(serverSocket, &clientIpAddr, &clientPort);

      //Make sure the socket handle is valid
      if(clientSocket != NULL)
      {
         //Debug message
         TRACE_INFO("Jack Connection #%u established with client %s port %" PRIu16 "...\r\n",
            counter, ipAddrToString(&clientIpAddr, NULL), clientPort);

         //Create a task to service the client connection
         status = xTaskCreate(jackClientTask, "Jack Client", 1600, clientSocket,
            tskIDLE_PRIORITY + 1, &handle);

         //Did we encounter an error?
         if(status != pdPASS)
         {
            //Debug message
            TRACE_ERROR("Failed to create task!\r\n");

            //Close socket
            socketClose(clientSocket);
            //Release semaphore
            osReleaseSemaphore(&connectionSemaphore);
         }
      }
   }
}


/**
 * @brief Jack client task
 * @param param[in] Client socket
 **/

void jackClientTask(void *param)
{
	error_t error;
	error_t local_err;
	size_t n;
	Socket *clientSocket;
	TlsContext *tlsContext;
	char_t buffer[512];
	TickType_t wakeTimer = 0, wakeTimerPrev = 0;
	uint32_t rxBufferIndex = 0;
	uint32_t timeslice_ms = 0;

	//Variable initialization
	tlsContext = NULL;

	//Retrieve socket handle
	clientSocket = (Socket *) param;

	jackSocketHandle = clientSocket;
	//Start of exception handling block
	do
	{
		//Set timeout
		error = socketSetTimeout(clientSocket, APP_SERVER_TIMEOUT);

		// Set tcp keepalive option
		int keepAlive = 1;	// 1= enable keep alive
		int keepIdle = 5*1000;	// 5000 = send first keepalive probe after 5 seconds of idleness;
		int keepInterval = 3*1000;// 3000 = send subsequent keepalive probels after 3 seconds.
		int keepCount = 3; 	// 3 = timeout after 3 failed probes.;
		local_err = socketEnableKeepAlive(clientSocket, keepAlive);
		local_err = socketSetKeepAliveParams(clientSocket, keepIdle,
				keepInterval, keepCount);


		// enable send timeout.
//		struct timeval sendTimeout, recvTimeout;
//		sendTimeout.tv_sec = 2;
//		sendTimeout.tv_usec = 0;		// 2s
//		recvTimeout.tv_sec = 0;
//		recvTimeout.tv_usec = 10000;	//0.01 S
		//{
		//  long    tv_sec;         /* seconds */
		//  long    tv_usec;        /* and microseconds */
		//};
		//local_err = setsockopt(jackSocketHandle, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));	// enable send timeout.
		//local_err = setsockopt(jackSocketHandle, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(sendTimeout));	// enable rx timeout.

		//Any error to report?
		if(error)
			break;
#if APP_JACK_TLS
		//TLS context initialization
		tlsContext = tlsInit();
		//Failed to initialize TLS context?
		if(tlsContext == NULL)
		{
			//Report an error
			error = ERROR_OUT_OF_MEMORY;
			//Exit immediately
			break;
		}

		//Select server operation mode
		error = tlsSetConnectionEnd(tlsContext, TLS_CONNECTION_END_SERVER);
		//Any error to report?
		if(error)
			break;

		//Bind TLS to the relevant socket
		error = tlsSetSocket(tlsContext, clientSocket);
		//Any error to report?
		if(error)
			break;

		//Set TX and RX buffer size
		error = tlsSetBufferSize(tlsContext, 2048, 16384);
		//Any error to report?
		if(error)
			break;

		//Set the PRNG algorithm to be used
		error = tlsSetPrng(tlsContext, YARROW_PRNG_ALGO, &yarrowContext);
		//Any error to report?
		if(error)
			break;

		//Set supported TLS version(s)
		error = tlsSetVersion(tlsContext, TLS_VERSION_1_3, TLS_VERSION_1_3);
		//Any error to report?
		if(error)
			break;

		//Preferred cipher suite list
		error = tlsSetCipherSuites(tlsContext, cipherSuites,
				arraysize(cipherSuites));
		//Any error to report?
		if(error)
			break;

		//Register PSK callback function
		error = tlsSetPskCallback(tlsContext, tlsServerPskCallback);
		//Any error to report?
		if(error)
			break;

#if (TLS_TICKET_SUPPORT == ENABLED)
		//Enable session ticket mechanism
		error = tlsEnableSessionTickets(tlsContext, TRUE);
		//Any error to report?
		if(error)
			break;

		//Enable session ticket mechanism
		error = tlsSetTicketCallbacks(tlsContext, tlsEncryptTicket,
				tlsDecryptTicket, &tlsTicketContext);
		//Any error to report?
		if(error)
			break;
#endif

		//Establish a secure session
		error = tlsConnect(tlsContext);
		//TLS handshake failure?
		if(error)
			break;
#endif

		rxBufferIndex = 0;
		jackRXCount = 0;
		wdog_networkActivity |= 1;
		wakeTimer = xTaskGetTickCount();

		//Read HTTP request
		while(1)
		{






			do {
				//				vTaskDelay(75);
				// streaming messages will go here and will be organized by timeslice

				xSemaphoreTake( xJackConnectionMutex, ( TickType_t ) portMAX_DELAY );
				if((rxBufferIndex < 0) || (rxBufferIndex > 1460)){
					rxBufferIndex = 0;	// fault scenario.
					jackRXCount = 0;
				}
				if((jackRXCount < 0) || (jackRXCount > 1460)){
					rxBufferIndex = 0;	// fault scenario.
					jackRXCount = 0;
				}
				//Read a complete line
#if APP_JACK_TLS
				error = tlsRead(tlsContext, buffer, sizeof(buffer) - 1, &n,
						TLS_FLAG_BREAK_CRLF);
#else
				error = socketReceive(clientSocket, &(RXBuffer[rxBufferIndex]), sizeof(RXBuffer) - 1 - rxBufferIndex, &n,
						0);
#endif
				jackRXCount += n;
				//recv(jackSocketHandle, &(RXBuffer[rxBufferIndex]), sizeof(RXBuffer) - 1 - rxBufferIndex, 0);
				xSemaphoreGive( xJackConnectionMutex);
				if(jackRXCount < 0){
					if(error == ERROR_NOT_CONNECTED){
						local_err = -1;	// ENOTCONN means the other end has blown away the connection.  We need to reset.
						xSemaphoreTake( xPrintMutex, ( TickType_t ) portMAX_DELAY );
						PRINTF("Jack port closed: errno %d\r\n", error);
						xSemaphoreGive(xPrintMutex);
					}
					else if(error != ERROR_WOULD_BLOCK){		//EAGAIN could also be 'would block'
					}
				}


				uint64_t timestamp;
				int32_t msgLength;


				while(jackRXCount > 0){

					// check for alignment.
					if(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->STX !=  STX_VAL){
						// mis-aligned.
						rxBufferIndex++;
						if(rxBufferIndex >= (sizeof(RXBuffer) -1)){
							jackRXCount = -1;	// fault condition.
						}
						jackRXCount--;
						if(jackRXCount == 0){
							rxBufferIndex = 0;
						}
						continue;
					}

					msgLength = (((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->LEN +
							sizeof(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->LEN) +
							sizeof(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->STX)  );
					// check that the Length byte is reasonable.
					if((msgLength > ((JACK_BUFFER_SIZE) * 2 / 3)) || (msgLength < sizeof(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->LEN) + sizeof(((jack_header_t * )(&(RXBuffer[rxBufferIndex])))->STX)))
					{
						// Length byte is unreasonable.  This is an error.  Treat this as mis-alignment.
						rxBufferIndex++;
						if(rxBufferIndex >= (sizeof(RXBuffer) -1)){
							jackRXCount = -1;	// fault condition.
						}
						jackRXCount--;
						if(jackRXCount == 0){
							rxBufferIndex = 0;
						}
						continue;
					}
					// check that the entire msg is in the RXBuffer.
					if(msgLength > jackRXCount)
					{
						break;	// break loop so we can receive more bytes.
					}

					local_err = process_message(&(RXBuffer[rxBufferIndex]));

					if((local_err < 0) || (msgLength > jackRXCount)){
						jackRXCount = 0;
						//						local_err = -1;
						rxBufferIndex = 0;
						continue;
					}else if( msgLength > sizeof(RXBuffer) - 1 - rxBufferIndex){
						// this is an error condition.  I.e. an error in the process_message function.  we received too many bytes.
						jackRXCount = 0;
						local_err = -1;
						rxBufferIndex = 0;
						continue;
					}
					if(msgLength == jackRXCount){
						// we received all the bytes in the queue.  Reset the rxBufferIndex
						jackRXCount = jackRXCount - msgLength;
						local_err = 0;
						rxBufferIndex = 0;

					}else if(msgLength == 0){
						// we didn't receive any bytes.  Likely because the whole msg isn't in the receive buffer.
						// abort our loop and receive more bytes.
						local_err = 0;
						break;
					}else{
						// there's more bytes in the RXBuffer that need to be processed (complete additional msg or not).
						jackRXCount = jackRXCount - msgLength;
						local_err = 0;
						rxBufferIndex += msgLength;

					}
				}



				if (local_err != NO_ERROR) {
					PRINTF("Jack Client Task: error on Receive (err=%d)\r\n", local_err);
					//					continue;		// TODO handle this
					break;	// handled!
				}


				if(jackRXCount == 0){
					vTaskDelay(50);	// nothing to receive last time.
				}


				// streaming code //

#define TIME_BETWEEN_MESSAGES_mS	250

				wakeTimer = xTaskGetTickCount() / portTICK_PERIOD_MS;
				if(wakeTimer > wakeTimerPrev){
					timeslice_ms += wakeTimer - wakeTimerPrev;
				}else{
					timeslice_ms += (portMAX_DELAY - wakeTimerPrev) + wakeTimer;
				}
				wakeTimerPrev = wakeTimer;
				if(timeslice_ms > TIME_BETWEEN_MESSAGES_mS)
				{
					timeslice_ms = timeslice_ms - TIME_BETWEEN_MESSAGES_mS;
					// 2Hz
					if(diagnostic_change_flag){
						local_err |= jack_send_ucm_diagnostic(JACK_DIAG_SUBTYPE_1);
						if(local_err == NO_ERROR){
							// clear the diagnostic change flag so we don't keep sending diag messages
							diagnostic_change_flag = 0;
						}
					}

					if(active_streams & JACK_STREAM_PLOT_MASK){
						timestamp = app_jack_get_timestamp(JACK_MSG_TYPE_APP_UCM_PLOT);
						local_err |= jack_send_ucm_plot(timestamp);
					}else{
						timeslice_ms = 0;
					}
				}
				if(timeslice_ms > TIME_BETWEEN_MESSAGES_mS){
					timeslice_ms = TIME_BETWEEN_MESSAGES_mS;
				}
				// streaming code //

				if (local_err != NO_ERROR) {
					PRINTF("Jack Server Task: error on Receive (err=%d)\r\n", local_err);
					//					continue;		// TODO handle this
					break;	// handled!
				}

				if(local_err == NO_ERROR){
					wdog_networkActivity |= 1;
				}



			}while(local_err == NO_ERROR);

			//Any error to report?
			if(error)
				break;
		}

		//Propagate exception if necessary
		if(error)
			break;

		//Send response to the client
#if APP_JACK_TLS
//		error = tlsWrite(tlsContext, buffer, n, NULL, 0);
#else
//		error = socketSend(clientSocket, buffer, n, NULL, 0);
#endif
		//Any error to report?
//		if(error)
//			break;
#if APP_JACK_TLS
		//Terminate TLS session
		error = tlsShutdown(tlsContext);
		//Any error to report?
		if(error)
			break;
#endif
		//Graceful shutdown
//		error = socketShutdown(clientSocket, SOCKET_SD_BOTH);
		//Any error to report?
//		if(error)
//			break;

		//End of exception handling block
	} while(0);

#if APP_JACK_TLS
	//Release TLS context
	if(tlsContext != NULL)
	{
		tlsFree(tlsContext);
	}
#endif

	//Close socket
	if(clientSocket != NULL)
	{
		socketClose(clientSocket);
	}

	//Debug message
	TRACE_INFO("Connection closed...\r\n");

	//Release semaphore
	osReleaseSemaphore(&connectionSemaphore);

	//Kill ourselves
	osDeleteTask(OS_SELF_TASK_ID);
}


/**
 * @brief PSK callback function
 * @param[in] context Pointer to the TLS context
 * @param[in] pskIdentity PSK identity of the client
 * @param[in] pskIdentityLen Length of the PSK identity, in bytes
 * @return Error code
 **/

error_t tlsServerPskCallback(TlsContext *context, const uint8_t *pskIdentity,
   size_t pskIdentityLen)
{
   error_t error;

   //Debug message
   TRACE_INFO("TLS Server: PSK callback\r\n");

   //Check PSK identity
   if(pskIdentityLen == strlen(APP_CLIENT1_PSK_IDENTITY) &&
      !memcmp(pskIdentity, APP_CLIENT1_PSK_IDENTITY, pskIdentityLen))
   {
      //Set the pre-shared key to be used
      error = tlsSetPsk(context, client1Psk, sizeof(client1Psk));
   }
   else if(pskIdentityLen == strlen(APP_CLIENT2_PSK_IDENTITY) &&
      !memcmp(pskIdentity, APP_CLIENT2_PSK_IDENTITY, pskIdentityLen))
   {
      //Set the pre-shared key to be used
      error = tlsSetPsk(context, client2Psk, sizeof(client2Psk));
   }
   else
   {
      //Unknown PSK identity
      error = ERROR_UNKNOWN_IDENTITY;
   }

   //Return status code
   return error;
}


/**
 * @brief Display the contents of an array
 * @param[out] buffer Output buffer where to format the resulting string
 * @param[in] data Pointer to the data array
 * @param[in] length Number of bytes in the array
 * @return Length of the resulting string
 **/

size_t dumpArray(char_t *buffer, const uint8_t *data, size_t length)
{
   size_t i;
   size_t n;

   //Variable initialization
   n = 0;

   //Properly terminate the string
   buffer[0] = '\0';

   //Process input data
   for(i = 0; i < length; i++)
   {
      //Beginning of a new line?
      if(i != 0 && (i % 16) == 0)
      {
         n += sprintf(buffer + n, "\r\n        ");
      }

      //Display current data byte
      n += sprintf(buffer + n, "%02" PRIX8 " ", data[i]);
   }

   //Return the length of the resulting string
   return n;
}

