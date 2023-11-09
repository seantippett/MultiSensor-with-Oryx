/**
 * @file rstp_prs.h
 * @brief Port Role Selection state machine (PRS)
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

#ifndef _RSTP_PRS_H
#define _RSTP_PRS_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Port Role Selection machine states
 **/

typedef enum
{
   RSTP_PRS_STATE_INIT_BRIDGE    = 0,
   RSTP_PRS_STATE_ROLE_SELECTION = 1
} RstpPrsState;


//RSTP related functions
void rstpPrsInit(RstpBridgeContext *context);
void rstpPrsFsm(RstpBridgeContext *context);
void rstpPrsChangeState(RstpBridgeContext *context, RstpPrsState newState);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
