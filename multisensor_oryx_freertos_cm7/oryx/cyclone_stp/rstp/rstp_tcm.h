/**
 * @file rstp_tcm.h
 * @brief Topology change state machine (TCM)
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

#ifndef _RSTP_TCM_H
#define _RSTP_TCM_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Topology Change machine states
 **/

typedef enum
{
   RSTP_TCM_STATE_INACTIVE     = 0,
   RSTP_TCM_STATE_LEARNING     = 1,
   RSTP_TCM_STATE_DETECTED     = 2,
   RSTP_TCM_STATE_NOTIFIED_TCN = 3,
   RSTP_TCM_STATE_NOTIFIED_TC  = 4,
   RSTP_TCM_STATE_PROPAGATING  = 5,
   RSTP_TCM_STATE_ACKNOWLEDGED = 6,
   RSTP_TCM_STATE_ACTIVE       = 7
} RstpTcmState;


//RSTP related functions
void rstpTcmInit(RstpBridgePort *port);
void rstpTcmFsm(RstpBridgePort *port);
void rstpTcmChangeState(RstpBridgePort *port, RstpTcmState newState);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
