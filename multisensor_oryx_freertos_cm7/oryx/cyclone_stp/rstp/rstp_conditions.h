/**
 * @file rstp_conditions.h
 * @brief RSTP state machine conditions
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

#ifndef _RSTP_CONDITIONS_H
#define _RSTP_CONDITIONS_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//RSTP related functions
uint_t rstpAdminEdge(RstpBridgePort *port);
uint_t rstpAutoEdge(RstpBridgePort *port);
bool_t rstpAllSynced(RstpBridgeContext *context);
uint_t rstpEdgeDelay(RstpBridgePort *port);
uint_t rstpForwardDelay(RstpBridgePort *port);
uint_t rstpFwdDelay(RstpBridgePort *port);
uint_t rstpHelloTime(RstpBridgePort *port);
uint_t rstpMaxAge(RstpBridgePort *port);
uint_t rstpMigrateTime(RstpBridgeContext *context);
bool_t rstpReRooted(RstpBridgePort *port);
bool_t rstpVersion(RstpBridgeContext *context);
bool_t stpVersion(RstpBridgeContext *context);
uint_t rstpTxHoldCount(RstpBridgeContext *context);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
