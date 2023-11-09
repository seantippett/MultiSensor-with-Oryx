/**
 * @file rstp_misc.h
 * @brief RSTP helper functions
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

#ifndef _RSTP_MISC_H
#define _RSTP_MISC_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Parameter value/name binding
 **/

typedef struct
{
   uint_t value;
   const char_t *name;
} RstpParamName;


//RSTP related functions
void rstpLock(RstpBridgeContext *context);
void rstpUnlock(RstpBridgeContext *context);
void rstpTick(RstpBridgeContext *context);

RstpBridgePort *rstpGetBridgePort(RstpBridgeContext *context, uint16_t portId);

int_t rstpComparePortNum(uint16_t portId1, uint16_t portId2);
int_t rstpCompareBridgeAddr(const MacAddr *addr1, const MacAddr *addr2);
int_t rstpCompareBridgeId(const StpBridgeId *id1, const StpBridgeId *id2);
int_t rstpComparePriority(const RstpPriority *p1, const RstpPriority *p2);
int_t rstpCompareTimes(const RstpTimes *t1, const RstpTimes *t2);

void rstpUpdateTopologyChangeCount(RstpBridgeContext *context);
void rstpUpdatePortPathCost(RstpBridgePort *port);
void rstpUpdateOperPointToPointMac(RstpBridgePort *port);
void rstpUpdatePortState(RstpBridgePort *port, SwitchPortState state);
void rstpUpdateAgeingTime(RstpBridgeContext *context, uint32_t ageingTime);
void rstpEnableRsvdMcastTable(RstpBridgeContext *context, bool_t enable);

error_t rstpAddStaticFdbEntry(RstpBridgeContext *context, const MacAddr *macAddr,
   bool_t override);

error_t rstpDeleteStaticFdbEntry(RstpBridgeContext *context,
   const MacAddr *macAddr);

void rstpRemoveFdbEntries(RstpBridgePort *port);
void rstpFlushFdbTable(RstpBridgePort *port);

error_t rstpConfigurePermanentDatabase(RstpBridgeContext *context);
void rstpUnconfigurePermanentDatabase(RstpBridgeContext *context);

void rstpGeneratePortAddr(RstpBridgePort *port);

bool_t rstpCheckBridgeParams(uint_t maxAge, uint_t helloTime,
   uint_t forwardDelay);

const char_t *rstpGetParamName(uint_t value, const RstpParamName *paramList,
   size_t paramListLen);

void rstpDecrementTimer(uint_t *x);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
