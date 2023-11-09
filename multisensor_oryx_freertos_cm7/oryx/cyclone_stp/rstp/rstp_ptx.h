/**
 * @file rstp_ptx.h
 * @brief Port Transmit state machine (PTX)
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

#ifndef _RSTP_PTX_H
#define _RSTP_PTX_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Port Transmit machine states
 **/

typedef enum
{
   RSTP_PTX_STATE_TRANSMIT_INIT     = 0,
   RSTP_PTX_STATE_TRANSMIT_PERIODIC = 1,
   RSTP_PTX_STATE_TRANSMIT_CONFIG   = 2,
   RSTP_PTX_STATE_TRANSMIT_TCN      = 3,
   RSTP_PTX_STATE_TRANSMIT_RSTP     = 4,
   RSTP_PTX_STATE_IDLE              = 5
} RstpPtxState;


//RSTP related functions
void rstpPtxInit(RstpBridgePort *port);
void rstpPtxFsm(RstpBridgePort *port);
void rstpPtxChangeState(RstpBridgePort *port, RstpPtxState newState);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
