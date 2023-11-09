

#ifndef _APP_BOARD_H
#define _APP_BOARD_H

#if(0)


#pragma pack(1)
struct STR_PowerSubMsg_1{
	float inputVolts;
	float inputCurrent_A;
	float inputCurrent_B;
	enum POWER_STATE powerState;
	uint32_t bitField;
};





#pragma pack(1)
struct STR_POWER_MSG_Headder {
	uint32_t	STX;
	uint32_t	LEN;
	uint32_t	TYPE;
	uint32_t	VER;
	uint32_t	NODE;
	uint64_t	TIME;
	uint32_t	SUB_MSG_TYPE;
} ;

#pragma pack(1)
struct STR_POWER_MSG {
	struct STR_POWER_MSG_Headder msgHeadder;
	struct STR_PowerSubMsg_1 msg01;
};

#endif



#define DIGIPYRO_THRESHOLD	(uint32_t)100
	// theshold is 8 bit, and bits 17-24
	// first guess at threshold is half way (128).

#define DIGIPYRO_BLIND_TIME	(uint32_t)7
	// Blind Time is 4 bit, and bits 13-16
	// Blind time is 0.5s + 0.5 * BlindTime
	// It is the minimum time after one alarm to the next.
	// first guess is 4s

#define DIGIPYRO_PULSE_CNT	((uint32_t)0)
	// Pulse Count is 2 bits, at bits 11,12.
	// Pulse count is the number of counts over the threshold to
	// cause an alarm.  Formula is pulse count = 1 + PULSE_CNT

#define DIGIPYRO_WINDOW_TIME ((uint32_t)0)
	// Window Time is 2 bits, at bits 9,10.
	// Window time is used with pulse count.  It is the size (in seconds)
	// of the window in which pulse count counts a pulse.
	// Window Time (seconds) = 2+ 2* WINDOW_TIME

#define DIGIPYRO_OPP_MODE (uint32_t)0
	// Opp Mode is 2 bits at bits 7,8.
	// Opp Mode = 0 = "forced readout" mode (streaming).
	// Opp Mode = 1 = "Interrupt Readout" mode (just send me a flag on an event).
	// Opp Mode = 2 = "Wakeup Mode"
	// Opp Mode = 3 = Reserved.

#define DIGIPYRO_SIG_SOURCE (uint32_t)0
	// Signal Source is 2 bits, bits 5,6.
	// 0 sets the sig source to after BPF. (signal will be ranged of +/- 8192).
	// 1 sets the sig source to after LPF. (signal will be ranged to 0 to 16383.
	// 2 is reseved.
	// 3 reads the temperature.  In range of 0 to 16383 value.  Proportianl to temperature.
	// 			temperature can be used to ignore false triggers due to temperature changes.
	//			greater than 1 deg C per minute.

// BIT 3,4 is RESERVED

#define DIGIPYRO_HPF_CUTOFF (uint32_t)0
	// High Pass Filter Cutoff.  1 bit., Bit 2.
	// 0 = 0.4Hz.
	// 1 = 0.2Hz. (used for longer distance meausring).

// BIT 1 IS RESEVERED!!!

#define DIGIPYRO_COUNT_MODE (uint32_t)0
	// 1 bit, this is bit 0.
	// 0 = count pulse when sign of signal changes.
	// 1 = no zero crossing is required.

void DigiPyro_WriteSettings(uint32_t data);
void DigiPyro_initUART(void);


#pragma pack(1)// packing is LSB first.
struct STR_PIR_SL_MSG{					// serial Link msg format.
	uint8_t		countMode		:1;		//
	uint8_t		reserved_0x0	:1;		// must be set to 0x00
	uint8_t		HPFcutoff		:1;		//
	uint8_t		reserved_0x2	:2;		// must be set to 0x02
	uint8_t		signalSource	:2;
	uint8_t		operationMode	:2;
	uint8_t		windowTime		:2;
	uint8_t		pulseCounter  	:2;
	uint8_t		blindTime 	  	:4;
	uint8_t		threshold 		:8;
	uint8_t		stuffing		:7;
};


#pragma pack(1)// packing is LSB first.
struct STR_PIR_DL_MSG{
	uint8_t		countMode		:1;		//
	uint8_t		reserved_0x0	:1;		// must be set to 0x00
	uint8_t		HPFcutoff		:1;		//
	uint8_t		reserved_0x2	:2;		// must be set to 0x02
	uint8_t		signalSource	:2;
	uint8_t		operationMode	:2;
	uint8_t		windowTime		:2;
	uint8_t		pulseCounter  	:2;
	uint8_t		blindTime 	  	:4;
	uint8_t		threshold 		:8;
	int16_t	adcCounts		:14;
	uint8_t		outOfRange		:1;
	uint32_t	stuffing		:24;
};
struct STR_PIR_DL_MSG_uint{
	uint8_t		countMode		:1;		//
	uint8_t		reserved_0x0	:1;		// must be set to 0x00
	uint8_t		HPFcutoff		:1;		//
	uint8_t		reserved_0x2	:2;		// must be set to 0x02
	uint8_t		signalSource	:2;
	uint8_t		operationMode	:2;
	uint8_t		windowTime		:2;
	uint8_t		pulseCounter  	:2;
	uint8_t		blindTime 	  	:4;
	uint8_t		threshold 		:8;
	uint16_t adcCounts		:14;		/// NOTE!!! this handles ADC as unsigned.
	uint8_t		outOfRange		:1;
	uint32_t	stuffing		:24;
};

#pragma pack(1)
union U_PIR_MSG{
	uint64_t				raw;
	struct STR_PIR_DL_MSG 	DLmsg;
	struct STR_PIR_DL_MSG_uint 	DLmsg_uint;
	struct STR_PIR_SL_MSG	SLmsg	   	;
};





#pragma pack(1)
union U_PIR_SL_MSG{						// serial Link Msg format.
	uint32_t				raw			;
	struct STR_PIR_SL_MSG	SLmsg	   	;
};


#pragma pack(1)
union U_PIR_DL_MSG{
	uint64_t					raw		;
	struct STR_PIR_DL_MSG		DLmsg	;
};
#pragma pack()

extern bool sync_flag; // for the diagnostic msg
extern uint8_t sync_error_change;

extern uint64_t runtime_uS;
extern uint64_t currentTime_uS;
extern int32_t currentTimeIncrementationFactor;
extern void getHHMMSS(uint8_t *hh, uint8_t *mm, uint8_t *ss);
void resetBoard(void);
void sendTimeSync(void);








#endif
