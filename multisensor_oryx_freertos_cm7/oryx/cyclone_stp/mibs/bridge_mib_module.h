/**
 * @file bridge_mib_module.h
 * @brief Bridge MIB module
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

#ifndef _BRIDGE_MIB_MODULE_H
#define _BRIDGE_MIB_MODULE_H

//Dependencies
#include "mibs/mib_common.h"
#include "stp/stp.h"
#include "rstp/rstp.h"

//Bridge MIB module support
#ifndef BRIDGE_MIB_SUPPORT
   #define BRIDGE_MIB_SUPPORT DISABLED
#elif (BRIDGE_MIB_SUPPORT != ENABLED && BRIDGE_MIB_SUPPORT != DISABLED)
   #error BRIDGE_MIB_SUPPORT parameter is not valid
#endif

//Support for SET operations
#ifndef BRIDGE_MIB_SET_SUPPORT
   #define BRIDGE_MIB_SET_SUPPORT DISABLED
#elif (BRIDGE_MIB_SET_SUPPORT != ENABLED && BRIDGE_MIB_SET_SUPPORT != DISABLED)
   #error BRIDGE_MIB_SET_SUPPORT parameter is not valid
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Types of bridging
 **/

typedef enum
{
   BRIDGE_MIB_BASE_TYPE_UNKNOWN           = 1,
   BRIDGE_MIB_BASE_TYPE_TRANSPARENT_ONLY  = 2,
   BRIDGE_MIB_BASE_TYPE_SOURCE_ROUTE_ONLY = 3,
   BRIDGE_MIB_BASE_TYPE_SRT               = 4
} BridgeMibBaseType;


/**
 * @brief STP protocol specification
 **/

typedef enum
{
   BRIDGE_MIB_PROTOCOL_SPEC_UNKNOWN    = 1,
   BRIDGE_MIB_PROTOCOL_SPEC_DEC_LB100  = 2,
   BRIDGE_MIB_PROTOCOL_SPEC_IEEE802_1D = 3
} BridgeMibProtocolSpec;


/**
 * @brief Port state
 **/

typedef enum
{
   BRIDGE_MIB_PORT_STATE_UNKNOWN    = 0,
   BRIDGE_MIB_PORT_STATE_DISABLED   = 1,
   BRIDGE_MIB_PORT_STATE_BLOCKING   = 2,
   BRIDGE_MIB_PORT_STATE_LISTENING  = 3,
   BRIDGE_MIB_PORT_STATE_LEARNING   = 4,
   BRIDGE_MIB_PORT_STATE_FORWARDING = 5,
   BRIDGE_MIB_PORT_STATE_BROKEN     = 6
} BridgeMibPortState;


/**
 * @brief Status of the port
 **/

typedef enum
{
   BRIDGE_MIB_PORT_STATUS_ENABLED  = 1,
   BRIDGE_MIB_PORT_STATUS_DISABLED = 2
} BridgeMibPortStatus;


/**
 * @brief Status of forwarding database entry
 **/

typedef enum
{
   BRIDGE_MIB_FDB_STATUS_OTHER   = 1,
   BRIDGE_MIB_FDB_STATUS_INVALID = 2,
   BRIDGE_MIB_FDB_STATUS_LEARNED = 3,
   BRIDGE_MIB_FDB_STATUS_SELF    = 4,
   BRIDGE_MIB_FDB_STATUS_MGMT    = 5
} BridgeMibFdbStatus;


/**
 * @brief Status of static database entry
 **/

typedef enum
{
   BRIDGE_MIB_STATIC_STATUS_OTHER             = 1,
   BRIDGE_MIB_STATIC_STATUS_INVALID           = 2,
   BRIDGE_MIB_STATIC_STATUS_PERMANENT         = 3,
   BRIDGE_MIB_STATIC_STATUS_DELETE_ON_RESET   = 4,
   BRIDGE_MIB_STATIC_STATUS_DELETE_ON_TIMEOUT = 5
} BridgeMibStaticStatus;


/**
 * @brief Bridge MIB base
 **/

typedef struct
{
   NetInterface *interface;
#if (STP_SUPPORT == ENABLED)
   StpBridgeContext *stpBridgeContext;
#endif
#if (RSTP_SUPPORT == ENABLED)
   RstpBridgeContext *rstpBridgeContext;
#endif
   int32_t dot1dBaseType;
   int32_t dot1dStpProtocolSpecification;
   MacAddr dot1dStaticAddress;
   uint16_t dot1dStaticReceivePort;
   uint32_t dot1dStaticAllowedToGoTo;
} BridgeMibBase;


//Bridge MIB related constants
extern BridgeMibBase bridgeMibBase;
extern const MibObject bridgeMibObjects[];
extern const MibModule bridgeMibModule;

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
