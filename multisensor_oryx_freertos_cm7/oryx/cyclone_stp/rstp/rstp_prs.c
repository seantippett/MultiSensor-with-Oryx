/**
 * @file rstp_prs.c
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

//Switch to the appropriate trace level
#define TRACE_LEVEL RSTP_TRACE_LEVEL

//Dependencies
#include "rstp/rstp.h"
#include "rstp/rstp_fsm.h"
#include "rstp/rstp_prs.h"
#include "rstp/rstp_procedures.h"
#include "rstp/rstp_misc.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (RSTP_SUPPORT == ENABLED)

//PRS state machine's states
const RstpParamName rstpPrsStates[] =
{
   {RSTP_PRS_STATE_INIT_BRIDGE,    "INIT_BRIDGE"},
   {RSTP_PRS_STATE_ROLE_SELECTION, "ROLE_SELECTION"}
};


/**
 * @brief PRS state machine initialization
 * @param[in] context Pointer to the RSTP bridge context
 **/

void rstpPrsInit(RstpBridgeContext *context)
{
   //Enter initial state
   rstpPrsChangeState(context, RSTP_PRS_STATE_INIT_BRIDGE);
}


/**
 * @brief PRS state machine implementation
 * @param[in] context Pointer to the RSTP bridge context
 **/

void rstpPrsFsm(RstpBridgeContext *context)
{
   uint_t i;
   bool_t reselect;

   //All conditions for the current state are evaluated continuously until one
   //of the conditions is met (refer to IEEE Std 802.1D-2004, section 17.16)
   switch(context->prsState)
   {
   //INIT_BRIDGE state?
   case RSTP_PRS_STATE_INIT_BRIDGE:
      //Unconditional transition (UCT) to ROLE_SELECTION state
      rstpPrsChangeState(context, RSTP_PRS_STATE_ROLE_SELECTION);
      break;

   //ROLE_SELECTION state?
   case RSTP_PRS_STATE_ROLE_SELECTION:
      //Check whether the reselect variable is TRUE for any port
      for(reselect = FALSE, i = 0; i < context->numPorts; i++)
      {
         reselect |= context->ports[i].reselect;
      }

      //Whenever any bridge port's reselect variable is set by the Port
      //Information state machine, spanning tree information is recomputed
      if(reselect)
      {
         rstpPrsChangeState(context, RSTP_PRS_STATE_ROLE_SELECTION);
      }

      break;

   //Invalid state?
   default:
      //Just for sanity
      rstpFsmError(context);
      break;
   }
}


/**
 * @brief Update PRS state machine state
 * @param[in] context Pointer to the RSTP bridge context
 * @param[in] newState New state to switch to
 **/

void rstpPrsChangeState(RstpBridgeContext *context, RstpPrsState newState)
{
   //Dump the state transition
   TRACE_VERBOSE("PRS state machine %s -> %s\r\n",
      rstpGetParamName(context->prsState, rstpPrsStates, arraysize(rstpPrsStates)),
      rstpGetParamName(newState, rstpPrsStates, arraysize(rstpPrsStates)));

   //Switch to the new state
   context->prsState = newState;

   //On entry to a state, the procedures defined for the state are executed
   //exactly once (refer to IEEE Std 802.1D-2004, section 17.16)
   switch(context->prsState)
   {
   //INIT_BRIDGE state?
   case RSTP_PRS_STATE_INIT_BRIDGE:
      //On initialization all ports are assigned the Disabled port role
      rstpUpdtRoleDisabledTree(context);
      break;

   //ROLE_SELECTION state?
   case RSTP_PRS_STATE_ROLE_SELECTION:
      //Update spanning tree information and port roles
      rstpClearReselectTree(context);
      rstpUpdtRolesTree(context);
      rstpSetSelectedTree(context);
      break;

   //Invalid state?
   default:
      //Just for sanity
      break;
   }

   //The RSTP state machine is busy
   context->busy = TRUE;
}

#endif
