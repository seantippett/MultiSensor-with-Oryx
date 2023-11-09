/**
 * @file rstp_fsm.c
 * @brief Rapid Spanning Tree state machines
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
#include "rstp/rstp_misc.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (RSTP_SUPPORT == ENABLED)


/**
 * @brief RSTP state machine initialization
 * @param[in] context Pointer to the RSTP bridge context
 **/

void rstpFsmInit(RstpBridgeContext *context)
{
   uint_t i;
   RstpBridgePort *port;

   //The first (RootBridgeID) and third (DesignatedBridgeID) components of
   //the bridge priority vector are both equal to the value of the Bridge
   //Identifier. The other components are zero
   context->bridgePriority.rootBridgeId = context->bridgeId;
   context->bridgePriority.rootPathCost = 0;
   context->bridgePriority.designatedBridgeId = context->bridgeId;
   context->bridgePriority.designatedPortId = 0;
   context->bridgePriority.bridgePortId = 0;

   //BridgeTimes comprises four components (the current values of Bridge Forward
   //Delay, Bridge Hello Time, and Bridge Max Age, and a Message Age of zero)
   context->bridgeTimes.forwardDelay = context->params.bridgeForwardDelay;
   context->bridgeTimes.helloTime = context->params.bridgeHelloTime;
   context->bridgeTimes.maxAge = context->params.bridgeMaxAge;
   context->bridgeTimes.messageAge = 0;

   //Initialize bridge's root priority vector
   context->rootPriority = context->bridgePriority;
   //Initialize bridge's rootTimes parameter
   context->rootTimes = context->bridgeTimes;

   //The value of the ageingTime parameter is normally Ageing Time
   context->ageingTime = context->params.ageingTime;
   //Reset rapid ageing timer
   context->rapidAgeingWhile = 0;

   //Restore default ageing time
   rstpUpdateAgeingTime(context, context->params.ageingTime);

   //Clear BPDU
   osMemset(&context->bpdu, 0, sizeof(RstpBpdu));

   //Loop through the ports of the bridge
   for(i = 0; i < context->numPorts; i++)
   {
      //Point to the current bridge port
      port = &context->ports[i];

      //The designatedTimes for each port is set equal to the value of rootTimes
      //except for the Hello Time component, which is set equal to BridgeTimes'
      //Hello Time
      port->designatedTimes = context->rootTimes;
      port->designatedTimes.helloTime = context->bridgeTimes.helloTime;

      //Initialize msgPriority and msgTimes for each port
      memset(&port->msgPriority, 0, sizeof(RstpPriority));
      memset(&port->msgTimes, 0, sizeof(RstpTimes));

      //Reset parameters
      port->disputed = FALSE;
      port->rcvdInfo = RSTP_RCVD_INFO_OTHER;
      port->rcvdTc = FALSE;
      port->rcvdTcAck = FALSE;
      port->rcvdTcn = FALSE;
      port->tcProp = FALSE;
      port->updtInfo = FALSE;
   }

   //One instance of the Port Role Selection state machine is implemented for
   //the bridge
   rstpPrsInit(context);

   //One instance of each of the other state machines is implemented per port
   for(i = 0; i < context->numPorts; i++)
   {
      //Point to the current bridge port
      port = &context->ports[i];

      //Initialize Port Timers state machine
      rstpPtiInit(port);
      //Initialize Port Receive state machine
      rstpPrxInit(port);
      //Initialize Port Protocol Migration state machine
      rstpPpmInit(port);
      //Initialize Bridge Detection state machine
      rstpBdmInit(port);
      //Initialize Port Transmit state machine
      rstpPtxInit(port);
      //Initialize Port Information state machine
      rstpPimInit(port);
      //Initialize Port Role Transition state machine
      rstpPrtInit(port);
      //Initialize Port State Transition state machine
      rstpPstInit(port);
      //Initialize Topology Change state machine
      rstpTcmInit(port);
   }

   //Update RSTP state machine
   rstpFsm(context);
}


/**
 * @brief RSTP state machine implementation
 * @param[in] context Pointer to the RSTP bridge context
 **/

void rstpFsm(RstpBridgeContext *context)
{
   uint_t i;
   RstpBridgePort *port;

   //The behavior of an RSTP implementation in a bridge is specified by a
   //number of cooperating state machines
   do
   {
      //Clear the busy flag
      context->busy = FALSE;

      //A single Port Role Selection state machine shall be implemented per
      //bridge
      rstpPrsFsm(context);

      //One instance of each of the other state machines shall be implemented
      //per bridge port (refer to IEEE Std 802.1D-2004, section 17.15)
      for(i = 0; i < context->numPorts; i++)
      {
         //Point to the current bridge port
         port = &context->ports[i];

         //Port Timers state machine
         rstpPtiFsm(port);
         //Port Receive state machine
         rstpPrxFsm(port);
         //Port Protocol Migration state machine
         rstpPpmFsm(port);
         //Bridge Detection state machine
         rstpBdmFsm(port);
         //Port Information state machine
         rstpPimFsm(port);
         //Port Role Transition state machine
         rstpPrtFsm(port);
         //Port State Transition state machine
         rstpPstFsm(port);
         //Topology Change state machine
         rstpTcmFsm(port);

         //The fdbFlush flag is set by the topology change state machine to
         //instruct the filtering database to remove all entries for this port
         if(port->fdbFlush)
         {
            //Flush the filtering database for the specified port
            rstpRemoveFdbEntries(port);
         }
      }

      //Check whether the RSTP state machine is idle
      if(!context->busy)
      {
         //Loop through the ports of the bridge
         for(i = 0; i < context->numPorts; i++)
         {
            //Point to the current bridge port
            port = &context->ports[i];

            //Update Port Transmit state machine for each port
            rstpPtxFsm(port);
         }
      }

      //Transition conditions are evaluated continuously as long as the RSTP
      //state machine is busy
   } while(context->busy);
}


/**
 * @brief RSTP state machine error handler
 * @param[in] context Pointer to the RSTP bridge context
 **/

void rstpFsmError(RstpBridgeContext *context)
{
   //Debug message
   TRACE_ERROR("RSTP state machine error!\r\n");
}

#endif
