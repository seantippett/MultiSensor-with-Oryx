/**
 * @file bridge_mib_impl.h
 * @brief Bridge MIB module implementation (dot1dStp subtree)
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

#ifndef _BRIDGE_MIB_IMPL_STP_H
#define _BRIDGE_MIB_IMPL_STP_H

//Dependencies
#include "mibs/mib_common.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//Bridge MIB related functions
error_t bridgeMibGetDot1dStpProtocolSpecification(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibSetDot1dStpPriority(const MibObject *object, const uint8_t *oid,
   size_t oidLen, const MibVariant *value, size_t valueLen, bool_t commit);

error_t bridgeMibGetDot1dStpPriority(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpTimeSinceTopologyChange(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpTopChanges(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpDesignatedRoot(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpRootCost(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpRootPort(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpMaxAge(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpHelloTime(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpHoldTime(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpForwardDelay(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibSetDot1dStpBridgeMaxAge(const MibObject *object, const uint8_t *oid,
   size_t oidLen, const MibVariant *value, size_t valueLen, bool_t commit);

error_t bridgeMibGetDot1dStpBridgeMaxAge(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibSetDot1dStpBridgeHelloTime(const MibObject *object, const uint8_t *oid,
   size_t oidLen, const MibVariant *value, size_t valueLen, bool_t commit);

error_t bridgeMibGetDot1dStpBridgeHelloTime(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibSetDot1dStpBridgeForwardDelay(const MibObject *object, const uint8_t *oid,
   size_t oidLen, const MibVariant *value, size_t valueLen, bool_t commit);

error_t bridgeMibGetDot1dStpBridgeForwardDelay(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibSetDot1dStpPortEntry(const MibObject *object, const uint8_t *oid,
   size_t oidLen, const MibVariant *value, size_t valueLen, bool_t commit);

error_t bridgeMibGetDot1dStpPortEntry(const MibObject *object, const uint8_t *oid,
   size_t oidLen, MibVariant *value, size_t *valueLen);

error_t bridgeMibGetNextDot1dStpPortEntry(const MibObject *object, const uint8_t *oid,
   size_t oidLen, uint8_t *nextOid, size_t *nextOidLen);

error_t bridgeMibSetDot1dStpPortPriority(uint16_t portNum,
   const MibVariant *value, size_t valueLen, bool_t commit);

error_t bridgeMibGetDot1dStpPortPriority(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpPortState(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

error_t bridgeMibSetDot1dStpPortEnable(uint16_t portNum,
   const MibVariant *value, size_t valueLen, bool_t commit);

error_t bridgeMibGetDot1dStpPortEnable(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

error_t bridgeMibSetDot1dStpPortPathCost(uint16_t portNum,
   const MibVariant *value, size_t valueLen, bool_t commit);

error_t bridgeMibGetDot1dStpPortPathCost(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpPortDesignatedRoot(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpPortDesignatedCost(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpPortDesignatedBridge(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpPortDesignatedPort(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

error_t bridgeMibGetDot1dStpPortForwardTransitions(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

error_t bridgeMibSetDot1dStpPortPathCost32(uint16_t portNum,
   const MibVariant *value, size_t valueLen, bool_t commit);

error_t bridgeMibGetDot1dStpPortPathCost32(uint16_t portNum,
   MibVariant *value, size_t *valueLen);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
