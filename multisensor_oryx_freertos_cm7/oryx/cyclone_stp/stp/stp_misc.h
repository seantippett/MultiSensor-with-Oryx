/**
 * @file stp_misc.h
 * @brief STP helper functions
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

#ifndef _STP_MISC_H
#define _STP_MISC_H

//Dependencies
#include "stp/stp.h"

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
} StpParamName;


//STP related functions
void stpLock(StpBridgeContext *context);
void stpUnlock(StpBridgeContext *context);
void stpTick(StpBridgeContext *context);

StpBridgePort *stpGetBridgePort(StpBridgeContext *context, uint16_t portId);

int_t stpComparePortNum(uint16_t portId1, uint16_t portId2);
int_t stpCompareBridgeAddr(const MacAddr *addr1, const MacAddr *addr2);
int_t stpCompareBridgeId(const StpBridgeId *id1, const StpBridgeId *id2);

void stpUpdateTopologyChange(StpBridgeContext *context, bool_t value);
void stpUpdatePortState(StpBridgePort *port, StpPortState state);
void stpUpdateAgeingTime(StpBridgeContext *context, uint32_t ageingTime);
void stpEnableRsvdMcastTable(StpBridgeContext *context, bool_t enable);

error_t stpAddStaticFdbEntry(StpBridgeContext *context, const MacAddr *macAddr,
   bool_t override);

error_t stpDeleteStaticFdbEntry(StpBridgeContext *context,
   const MacAddr *macAddr);

error_t stpConfigurePermanentDatabase(StpBridgeContext *context);
void stpUnconfigurePermanentDatabase(StpBridgeContext *context);

void stpGeneratePortAddr(StpBridgePort *port);

bool_t stpCheckBridgeParams(uint_t maxAge, uint_t helloTime,
   uint_t forwardDelay);

const char_t *stpGetParamName(uint_t value, const StpParamName *paramList,
   size_t paramListLen);

void stpStartTimer(StpTimer *timer, uint_t value);
void stpStopTimer(StpTimer *timer);
bool_t stpIncrementTimer(StpTimer *timer, uint_t timeout);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
