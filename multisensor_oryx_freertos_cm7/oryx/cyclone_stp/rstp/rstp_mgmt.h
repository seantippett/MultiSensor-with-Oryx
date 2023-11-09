/**
 * @file rstp_mgmt.h
 * @brief Management of the RSTP bridge
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

#ifndef _RSTP_MGMT_H
#define _RSTP_MGMT_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//RSTP related functions
error_t rstpMgmtSetVersion(RstpBridgeContext *context, uint_t value,
   bool_t commit);

error_t rstpMgmtSetBridgePriority(RstpBridgeContext *context, uint16_t value,
   bool_t commit);

error_t rstpMgmtSetBridgeMaxAge(RstpBridgeContext *context, uint_t value,
   bool_t commit);

error_t rstpMgmtSetBridgeHelloTime(RstpBridgeContext *context, uint_t value,
   bool_t commit);

error_t rstpMgmtSetBridgeForwardDelay(RstpBridgeContext *context, uint_t value,
   bool_t commit);

error_t rstpMgmtSetTxHoldCount(RstpBridgeContext *context, uint_t value,
   bool_t commit);

error_t rstpMgmtSetAgeingTime(RstpBridgeContext *context, uint_t value,
   bool_t commit);

error_t rstpMgmtGetNumPorts(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetVersion(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetBridgeAddr(RstpBridgeContext *context, MacAddr *value);
error_t rstpMgmtGetBridgePriority(RstpBridgeContext *context, uint16_t *value);
error_t rstpMgmtGetBridgeMaxAge(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetBridgeHelloTime(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetBridgeForwardDelay(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetTxHoldCount(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetAgeingTime(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetDesignatedRoot(RstpBridgeContext *context, StpBridgeId *value);
error_t rstpMgmtGetRootPathCost(RstpBridgeContext *context, uint32_t *value);
error_t rstpMgmtGetRootPort(RstpBridgeContext *context, uint16_t *value);
error_t rstpMgmtGetMaxAge(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetHelloTime(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetForwardDelay(RstpBridgeContext *context, uint_t *value);
error_t rstpMgmtGetTopologyChanges(RstpBridgeContext *context, uint_t *value);

error_t rstpMgmtGetTimeSinceTopologyChange(RstpBridgeContext *context,
   uint_t *value);

error_t rstpMgmtSetPortPriority(RstpBridgeContext *context, uint_t portIndex,
   uint8_t value, bool_t commit);

error_t rstpMgmtSetAdminPortState(RstpBridgeContext *context, uint_t portIndex,
   bool_t value, bool_t commit);

error_t rstpMgmtSetAdminPortPathCost(RstpBridgeContext *context, uint_t portIndex,
   uint32_t value, bool_t commit);

error_t rstpMgmtSetAdminPointToPointMac(RstpBridgeContext *context,
   uint_t portIndex, RstpAdminPointToPointMac value, bool_t commit);

error_t rstpMgmtSetAdminEdgePort(RstpBridgeContext *context, uint_t portIndex,
   bool_t value, bool_t commit);

error_t rstpMgmtSetAutoEdgePort(RstpBridgeContext *context, uint_t portIndex,
   bool_t value, bool_t commit);

error_t rstpMgmtSetProtocolMigration(RstpBridgeContext *context, uint_t portIndex,
   bool_t value, bool_t commit);

error_t rstpMgmtGetPortAddr(RstpBridgeContext *context, uint_t portIndex,
   MacAddr *value);

error_t rstpMgmtGetPortPriority(RstpBridgeContext *context, uint_t portIndex,
   uint8_t *value);

error_t rstpMgmtGetAdminPortState(RstpBridgeContext *context, uint_t portIndex,
   bool_t *value);

error_t rstpMgmtGetMacOperState(RstpBridgeContext *context, uint_t portIndex,
   bool_t *value);

error_t rstpMgmtGetAdminPortPathCost(RstpBridgeContext *context,
   uint_t portIndex, uint32_t *value);

error_t rstpMgmtGetPortPathCost(RstpBridgeContext *context, uint_t portIndex,
   uint32_t *value);

error_t rstpMgmtGetAdminPointToPointMac(RstpBridgeContext *context,
   uint_t portIndex, RstpAdminPointToPointMac *value);

error_t rstpMgmtGetOperPointToPointMac(RstpBridgeContext *context,
   uint_t portIndex, bool_t *value);

error_t rstpMgmtGetAdminEdgePort(RstpBridgeContext *context, uint_t portIndex,
   bool_t *value);

error_t rstpMgmtGetAutoEdgePort(RstpBridgeContext *context, uint_t portIndex,
   bool_t *value);

error_t rstpMgmtGetOperEdgePort(RstpBridgeContext *context, uint_t portIndex,
   bool_t *value);

error_t rstpMgmtGetPortState(RstpBridgeContext *context, uint_t portIndex,
   StpPortState *value);

error_t rstpMgmtGetPortRole(RstpBridgeContext *context, uint_t portIndex,
   StpPortRole *value);

error_t rstpMgmtGetPortDesignatedRoot(RstpBridgeContext *context,
   uint_t portIndex, StpBridgeId *value);

error_t rstpMgmtGetPortDesignatedCost(RstpBridgeContext *context,
   uint_t portIndex, uint32_t *value);

error_t rstpMgmtGetPortDesignatedBridge(RstpBridgeContext *context,
   uint_t portIndex, StpBridgeId *value);

error_t rstpMgmtGetPortDesignatedPort(RstpBridgeContext *context,
   uint_t portIndex, uint16_t *value);

error_t rstpMgmtGetForwardTransitions(RstpBridgeContext *context,
   uint_t portIndex, uint_t *value);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
