/**
 * @file stp_bpdu.h
 * @brief BPDU processing
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneSTP Eval.
 *
 * This software is provided in source form for a short-term evaluation only. The
 * evaluation license expires 90 days after the date you first download the software.
 *
 * If you plan to use this software in a commercial product, you are required to
 * purchase a commercial license from Oryx Embedded SARL.
 *
 * After the 90-day evaluation period, you agree to either purchase a commercial
 * license or delete all copies of this software. If you wish to extend the
 * evaluation period, you must contact sales@oryx-embedded.com.
 *
 * This evaluation software is provided "as is" without warranty of any kind.
 * Technical support is available as an option during the evaluation period.
 *
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

#ifndef _STP_BPDU_H
#define _STP_BPDU_H

//Dependencies
#include "stp/stp.h"

//Size of BPDUs
#define STP_TCN_BPDU_SIZE    4
#define STP_CONFIG_BPDU_SIZE 35

//Port identifier field
#define STP_PORT_PRIORITY_MASK 0xFF00
#define STP_PORT_NUM_MASK      0x00FF

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief BPDU types
 **/

typedef enum
{
   STP_BPDU_TYPE_CONFIG = 0x00,
   STP_BPDU_TYPE_TCN    = 0x80,
} StpBpduTypes;


/**
 * @brief BPDU flags
 **/

typedef enum
{
   STP_BPDU_FLAG_TC     = 0x01,
   STP_BPDU_FLAG_TC_ACK = 0x80
} StpBpduFlags;


//CodeWarrior or Win32 compiler?
#if defined(__CWCC__) || defined(_WIN32)
   #pragma pack(push, 1)
#endif


/**
 * @brief Spanning Tree BPDU
 **/

typedef __packed_struct
{
   uint16_t protocolId;       //0-1
   uint8_t protocolVersionId; //2
   uint8_t bpduType;          //3
   uint8_t flags;             //4
   StpBridgeId rootId;       //5-12
   uint32_t rootPathCost;     //13-16
   StpBridgeId bridgeId;     //17-24
   uint16_t portId;           //25-26
   uint16_t messageAge;       //27-28
   uint16_t maxAge;           //29-30
   uint16_t helloTime;        //31-32
   uint16_t forwardDelay;     //33-34
} StpBpdu;


//CodeWarrior or Win32 compiler?
#if defined(__CWCC__) || defined(_WIN32)
   #pragma pack(pop)
#endif

//Bridge group address
extern const MacAddr STP_BRIDGE_GROUP_ADDR;

//STP related functions
void stpProcessLlcFrame(NetInterface *interface, EthHeader *ethHeader,
   const uint8_t *data, size_t length, NetRxAncillary *ancillary, void *param);

error_t stpProcessBpdu(StpBridgePort *port, const StpBpdu *bpdu,
   size_t length);

error_t stpSendBpdu(StpBridgePort *port, const StpBpdu *bpdu,
   size_t length);

error_t stpDumpBpdu(const StpBpdu *bpdu, size_t length);
void stpDumpFlags(uint8_t flags);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
