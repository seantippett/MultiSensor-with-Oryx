/**
 * @file rstp_prx.h
 * @brief Port Receive state machine (PRX)
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

#ifndef _RSTP_PRX_H
#define _RSTP_PRX_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Port Receive machine states
 **/

typedef enum
{
   RSTP_PRX_STATE_DISCARD = 0,
   RSTP_PRX_STATE_RECEIVE = 1
} RstpPrxState;


//RSTP related functions
void rstpPrxInit(RstpBridgePort *port);
void rstpPrxFsm(RstpBridgePort *port);
void rstpPrxChangeState(RstpBridgePort *port, RstpPrxState newState);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
