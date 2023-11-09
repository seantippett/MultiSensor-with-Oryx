/**
 * @file rstp_prt.h
 * @brief Port Role Transition state machine (PRT)
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

#ifndef _RSTP_PRT_H
#define _RSTP_PRT_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Port Role Transition machine states
 **/

typedef enum
{
   RSTP_PRT_STATE_INIT_PORT          = 0,
   RSTP_PRT_STATE_DISABLE_PORT       = 1,
   RSTP_PRT_STATE_DISABLED_PORT      = 2,
   RSTP_PRT_STATE_ROOT_PROPOSED      = 3,
   RSTP_PRT_STATE_ROOT_AGREED        = 4,
   RSTP_PRT_STATE_REROOT             = 5,
   RSTP_PRT_STATE_ROOT_FORWARD       = 6,
   RSTP_PRT_STATE_ROOT_LEARN         = 7,
   RSTP_PRT_STATE_REROOTED           = 8,
   RSTP_PRT_STATE_ROOT_PORT          = 9,
   RSTP_PRT_STATE_DESIGNATED_PROPOSE = 10,
   RSTP_PRT_STATE_DESIGNATED_SYNCED  = 11,
   RSTP_PRT_STATE_DESIGNATED_RETIRED = 12,
   RSTP_PRT_STATE_DESIGNATED_FORWARD = 13,
   RSTP_PRT_STATE_DESIGNATED_LEARN   = 14,
   RSTP_PRT_STATE_DESIGNATED_DISCARD = 15,
   RSTP_PRT_STATE_DESIGNATED_PORT    = 16,
   RSTP_PRT_STATE_ALTERNATE_PROPOSED = 17,
   RSTP_PRT_STATE_ALTERNATE_AGREED   = 18,
   RSTP_PRT_STATE_BLOCK_PORT         = 19,
   RSTP_PRT_STATE_BACKUP_PORT        = 20,
   RSTP_PRT_STATE_ALTERNATE_PORT     = 21
} RstpPrtState;


//RSTP related functions
void rstpPrtInit(RstpBridgePort *port);
void rstpPrtFsm(RstpBridgePort *port);

void rstpPrtDisabledPortFsm(RstpBridgePort *port);
void rstpPrtDisabledPortChangeState(RstpBridgePort *port, RstpPrtState newState);

void rstpPrtRootPortFsm(RstpBridgePort *port);
void rstpPrtRootPortChangeState(RstpBridgePort *port, RstpPrtState newState);

void rstpPrtDesignatedPortFsm(RstpBridgePort *port);
void rstpPrtDesignatedPortChangeState(RstpBridgePort *port, RstpPrtState newState);

void rstpPrtAlternatePortFsm(RstpBridgePort *port);
void rstpPrtAlternatePortChangeState(RstpBridgePort *port, RstpPrtState newState);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
