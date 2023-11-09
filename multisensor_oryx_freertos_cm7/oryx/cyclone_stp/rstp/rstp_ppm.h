/**
 * @file rstp_ppm.h
 * @brief Port Protocol Migration state machine (PPM)
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

#ifndef _RSTP_PPM_H
#define _RSTP_PPM_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Port Protocol Migration states
 **/

typedef enum
{
   RSTP_PPM_STATE_CHECKING_RSTP = 0,
   RSTP_PPM_STATE_SELECTING_STP = 1,
   RSTP_PPM_STATE_SENSING       = 2
} RstpPpmState;


//RSTP related functions
void rstpPpmInit(RstpBridgePort *port);
void rstpPpmFsm(RstpBridgePort *port);
void rstpPpmChangeState(RstpBridgePort *port, RstpPpmState newState);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
