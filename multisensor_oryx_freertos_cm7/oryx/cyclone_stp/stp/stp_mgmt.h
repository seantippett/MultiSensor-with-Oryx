/**
 * @file stp_mgmt.h
 * @brief Management of the STP bridge
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

#ifndef _STP_MGMT_H
#define _STP_MGMT_H

//Dependencies
#include "stp/stp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//STP related functions
error_t stpMgmtSetBridgePriority(StpBridgeContext *context, uint16_t value,
   bool_t commit);

error_t stpMgmtSetBridgeMaxAge(StpBridgeContext *context, uint_t value,
   bool_t commit);

error_t stpMgmtSetBridgeHelloTime(StpBridgeContext *context, uint_t value,
   bool_t commit);

error_t stpMgmtSetBridgeForwardDelay(StpBridgeContext *context, uint_t value,
   bool_t commit);

error_t stpMgmtSetAgeingTime(StpBridgeContext *context, uint_t value,
   bool_t commit);

error_t stpMgmtGetNumPorts(StpBridgeContext *context, uint_t *value);
error_t stpMgmtGetBridgeAddr(StpBridgeContext *context, MacAddr *value);
error_t stpMgmtGetBridgePriority(StpBridgeContext *context, uint16_t *value);
error_t stpMgmtGetBridgeMaxAge(StpBridgeContext *context, uint_t *value);
error_t stpMgmtGetBridgeHelloTime(StpBridgeContext *context, uint_t *value);
error_t stpMgmtGetBridgeForwardDelay(StpBridgeContext *context, uint_t *value);
error_t stpMgmtGetHoldTime(StpBridgeContext *context, uint_t *value);
error_t stpMgmtGetAgeingTime(StpBridgeContext *context, uint_t *value);
error_t stpMgmtGetDesignatedRoot(StpBridgeContext *context, StpBridgeId *value);
error_t stpMgmtGetRootPathCost(StpBridgeContext *context, uint32_t *value);
error_t stpMgmtGetRootPort(StpBridgeContext *context, uint16_t *value);
error_t stpMgmtGetMaxAge(StpBridgeContext *context, uint_t *value);
error_t stpMgmtGetHelloTime(StpBridgeContext *context, uint_t *value);
error_t stpMgmtGetForwardDelay(StpBridgeContext *context, uint_t *value);
error_t stpMgmtGetTopologyChanges(StpBridgeContext *context, uint_t *value);

error_t stpMgmtGetTimeSinceTopologyChange(StpBridgeContext *context,
   uint_t *value);

error_t stpMgmtSetPortPriority(StpBridgeContext *context, uint_t portIndex,
   uint8_t value, bool_t commit);

error_t stpMgmtSetAdminPortState(StpBridgeContext *context, uint_t portIndex,
   bool_t value, bool_t commit);

error_t stpMgmtSetPortPathCost(StpBridgeContext *context, uint_t portIndex,
   uint32_t value, bool_t commit);

error_t stpMgmtGetPortAddr(StpBridgeContext *context, uint_t portIndex,
   MacAddr *value);

error_t stpMgmtGetPortPriority(StpBridgeContext *context, uint_t portIndex,
   uint8_t *value);

error_t stpMgmtGetAdminPortState(StpBridgeContext *context, uint_t portIndex,
   bool_t *value);

error_t stpMgmtGetMacOperState(StpBridgeContext *context, uint_t portIndex,
   bool_t *value);

error_t stpMgmtGetPortPathCost(StpBridgeContext *context, uint_t portIndex,
   uint32_t *value);

error_t stpMgmtGetPortState(StpBridgeContext *context, uint_t portIndex,
   StpPortState *value);

error_t stpMgmtGetPortRole(StpBridgeContext *context, uint_t portIndex,
   StpPortRole *value);

error_t stpMgmtGetPortDesignatedRoot(StpBridgeContext *context,
   uint_t portIndex, StpBridgeId *value);

error_t stpMgmtGetPortDesignatedCost(StpBridgeContext *context,
   uint_t portIndex, uint32_t *value);

error_t stpMgmtGetPortDesignatedBridge(StpBridgeContext *context,
   uint_t portIndex, StpBridgeId *value);

error_t stpMgmtGetPortDesignatedPort(StpBridgeContext *context,
   uint_t portIndex, uint16_t *value);

error_t stpMgmtGetForwardTransitions(StpBridgeContext *context,
   uint_t portIndex, uint_t *value);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
