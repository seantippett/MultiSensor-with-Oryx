/**
 * @file rstp_bdm.c
 * @brief Bridge Detection state machine (BDM)
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

//Switch to the appropriate trace level
#define TRACE_LEVEL RSTP_TRACE_LEVEL

//Dependencies
#include "rstp/rstp.h"
#include "rstp/rstp_fsm.h"
#include "rstp/rstp_bdm.h"
#include "rstp/rstp_conditions.h"
#include "rstp/rstp_misc.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (RSTP_SUPPORT == ENABLED)

//BDM state machine's states
const RstpParamName rstpBdmStates[] =
{
   {RSTP_BDM_STATE_EDGE,     "EDGE"},
   {RSTP_BDM_STATE_NOT_EDGE, "NOT_EDGE"}
};


/**
 * @brief BDM state machine initialization
 * @param[in] port Pointer to the bridge port context
 **/

void rstpBdmInit(RstpBridgePort *port)
{
   //Enter initial state
   if(rstpAdminEdge(port))
   {
      rstpBdmChangeState(port, RSTP_BDM_STATE_EDGE);
   }
   else
   {
      rstpBdmChangeState(port, RSTP_BDM_STATE_NOT_EDGE);
   }
}


/**
 * @brief BDM state machine implementation
 * @param[in] port Pointer to the bridge port context
 **/

void rstpBdmFsm(RstpBridgePort *port)
{
   //All conditions for the current state are evaluated continuously until one
   //of the conditions is met (refer to IEEE Std 802.1D-2004, section 17.16)
   switch(port->bdmState)
   {
   //EDGE state?
   case RSTP_BDM_STATE_EDGE:
      //Evaluate conditions for the current state
      if((!port->portEnabled && !rstpAdminEdge(port)) || !port->operEdge)
      {
         //Switch to NOT_EDGE state
         rstpBdmChangeState(port, RSTP_BDM_STATE_NOT_EDGE);
      }

      break;

   //NOT_EDGE state?
   case RSTP_BDM_STATE_NOT_EDGE:
      //Evaluate conditions for the current state
      if((!port->portEnabled && rstpAdminEdge(port)) ||
         (port->edgeDelayWhile == 0 && rstpAutoEdge(port) &&
         port->sendRstp && port->proposing))
      {
         //Switch to EDGE state
         rstpBdmChangeState(port, RSTP_BDM_STATE_EDGE);
      }

      break;

   //Invalid state?
   default:
      //Just for sanity
      rstpFsmError(port->context);
      break;
   }
}


/**
 * @brief Update BDM state machine state
 * @param[in] port Pointer to the bridge port context
 * @param[in] newState New state to switch to
 **/

void rstpBdmChangeState(RstpBridgePort *port, RstpBdmState newState)
{
   //Dump the state transition
   TRACE_VERBOSE("Port %" PRIu8 ": BDM state machine %s -> %s\r\n",
      port->portIndex,
      rstpGetParamName(port->bdmState, rstpBdmStates, arraysize(rstpBdmStates)),
      rstpGetParamName(newState, rstpBdmStates, arraysize(rstpBdmStates)));

   //Switch to the new state
   port->bdmState = newState;

   //On entry to a state, the procedures defined for the state are executed
   //exactly once (refer to IEEE Std 802.1D-2004, section 17.16)
   switch(port->bdmState)
   {
   //EDGE state?
   case RSTP_BDM_STATE_EDGE:
      //Set operEdge flag
      port->operEdge = TRUE;
      break;

   //NOT_EDGE state?
   case RSTP_BDM_STATE_NOT_EDGE:
      //Clear operEdge flag
      port->operEdge = FALSE;
      break;

   //Invalid state?
   default:
      //Just for sanity
      break;
   }

   //The RSTP state machine is busy
   port->context->busy = TRUE;
}

#endif
