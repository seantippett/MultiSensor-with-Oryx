/**
 * @file stp_common.h
 * @brief STP common definitions
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

#ifndef _STP_COMMON_H
#define _STP_COMMON_H

//Dependencies
#include "stp_config.h"
#include "core/net.h"


/*
 * If you fully understand and accept the terms of the evalution license, then
 * edit the os_port_config.h header and add the following directive:
 *
 * #define EVAL_LICENSE_TERMS_ACCEPTED
 */

#ifndef EVAL_LICENSE_TERMS_ACCEPTED
   #error Before compiling CycloneSTP Eval, you must accept the terms of the evaluation license
#endif

//Version string
#define CYCLONE_STP_VERSION_STRING "2.3.2"
//Major version
#define CYCLONE_STP_MAJOR_VERSION 2
//Minor version
#define CYCLONE_STP_MINOR_VERSION 3
//Revision number
#define CYCLONE_STP_REV_NUMBER 2

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//Protocol identifier
#define STP_PROTOCOL_ID 0

//LLC header fields
#define STP_LLC_DSAP 0x42
#define STP_LLC_SSAP 0x42
#define STP_LLC_CTRL 0x03

//Minimum size of BPDUs
#define STP_MIN_BPDU_SIZE 4


/**
 * @brief Protocol versions
 **/

typedef enum
{
   STP_PROTOCOL_VERSION  = 0, ///<STP version
   RSTP_PROTOCOL_VERSION = 2, ///<RSTP version
   MSTP_PROTOCOL_VERSION = 3  ///<MSTP version
} StpProtocolVersion;


/**
 * @brief Port states
 **/

typedef enum
{
   STP_PORT_STATE_DISABLED   = 0,
   STP_PORT_STATE_BROKEN     = 1,
   STP_PORT_STATE_BLOCKING   = 2,
   STP_PORT_STATE_LISTENING  = 3,
   STP_PORT_STATE_LEARNING   = 4,
   STP_PORT_STATE_FORWARDING = 5
} StpPortState;


/**
 * @brief Port role values
 **/

typedef enum
{
   STP_PORT_ROLE_DISABLED   = 0,
   STP_PORT_ROLE_ROOT       = 1,
   STP_PORT_ROLE_DESIGNATED = 2,
   STP_PORT_ROLE_ALTERNATE  = 3,
   STP_PORT_ROLE_BACKUP     = 4
} StpPortRole;


//CodeWarrior or Win32 compiler?
#if defined(__CWCC__) || defined(_WIN32)
   #pragma pack(push, 1)
#endif


/**
 * @brief Bridge identifier
 **/

typedef __packed_struct
{
   uint16_t priority; //0-1
   MacAddr addr;      //2-7
} StpBridgeId;


//CodeWarrior or Win32 compiler?
#if defined(__CWCC__) || defined(_WIN32)
   #pragma pack(pop)
#endif

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
