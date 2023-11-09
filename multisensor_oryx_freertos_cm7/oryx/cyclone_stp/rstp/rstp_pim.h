/**
 * @file rstp_pim.h
 * @brief Port Information state machine (PIM)
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

#ifndef _RSTP_PIM_H
#define _RSTP_PIM_H

//Dependencies
#include "rstp/rstp.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Port Information machine states
 **/

typedef enum
{
   RSTP_PIM_STATE_DISABLED            = 0,
   RSTP_PIM_STATE_AGED                = 1,
   RSTP_PIM_STATE_UPDATE              = 2,
   RSTP_PIM_STATE_SUPERIOR_DESIGNATED = 3,
   RSTP_PIM_STATE_REPEATED_DESIGNATED = 4,
   RSTP_PIM_STATE_INFERIOR_DESIGNATED = 5,
   RSTP_PIM_STATE_NOT_DESIGNATED      = 6,
   RSTP_PIM_STATE_OTHER               = 7,
   RSTP_PIM_STATE_CURRENT             = 8,
   RSTP_PIM_STATE_RECEIVE             = 9
} RstpPimState;


//RSTP related functions
void rstpPimInit(RstpBridgePort *port);
void rstpPimFsm(RstpBridgePort *port);
void rstpPimChangeState(RstpBridgePort *port, RstpPimState newState);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
