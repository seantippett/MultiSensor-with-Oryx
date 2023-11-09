/**
 * @file stp_operation.c
 * @brief Operation of the protocol
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
#define TRACE_LEVEL STP_TRACE_LEVEL

//Dependencies
#include "stp/stp.h"
#include "stp/stp_operation.h"
#include "stp/stp_procedures.h"
#include "stp/stp_conditions.h"
#include "stp/stp_misc.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (STP_SUPPORT == ENABLED)


/**
 * @brief Received Configuration BPDU (8.7.1)
 * @param[in] port Pointer to the bridge port context
 * @param[in] bpdu Pointer to the received Configuration BPDU
 **/

void stpReceivedConfigBpdu(StpBridgePort *port, const StpBpdu *bpdu)
{
   bool_t rootBridge;
   StpBridgeContext *context;

   //Point to the STP bridge context
   context = port->context;

   //Check port state
   if(port->state != STP_PORT_STATE_DISABLED && port->macOperState)
   {
      //Check if the Configuration BPDU received conveys protocol information
      //that supersedes that already held for the port
      if(stpSupersedesPortInfo(port, bpdu))
      {
         //Test whether the bridge is currently the Root
         rootBridge = stpRootBridge(context);

         //The Record Configuration Information procedure is used
         stpRecordConfigInfo(port, bpdu);
         //The Configuration Update procedure is used
         stpConfigUpdate(context);
         //The Port State Selection procedure is used
         stpPortStateSelection(context);

         //Check if the Bridge was selected as the Root prior to Configuration
         //Update, but is no longer
         if(rootBridge && !stpRootBridge(context))
         {
            //The Hello Timer is stopped
            stpStopTimer(&context->helloTimer);

            //Check if the Topology Change Detected flag parameter is set
            if(context->topologyChangeDetected)
            {
               //The Topology Change Timer is stopped, the Transmit Topology
               //Change Notification BPDU procedure is used, and the Topology
               //Change Notification Timer is started
               stpStopTimer(&context->topologyChangeTimer);
               stpTransmitTcnBpdu(context);
               stpStartTimer(&context->tcnTimer, 0);
            }
         }

         //Check if the Configuration BPDU was received on the Root port
         if(stpRootPort(port))
         {
            //The Record Configuration Timeout Values and the Configuration
            //BPDU Generation (8.6.4) procedures are used
            stpRecordConfigTimeoutValues(context, bpdu);
            stpConfigBpduGeneration(context);

            //Check if the Topology Change Acknowledgment flag parameter is set
            if((bpdu->flags & STP_BPDU_FLAG_TC_ACK) != 0)
            {
               //The Topology Change Acknowledged procedure is used
               stpTopologyChangeAcked(context);
            }
         }
      }
      else
      {
         //If the Configuration BPDU received does not convey information
         //superseding that already held for the port and that port is the
         //Designated port for the LAN to which it is attached, then the Reply
         //to Configuration BPDU procedure is used
         if(stpDesignatedPort(port))
         {
            stpReplyToConfigBpdu(port);
         }
      }
   }
}


/**
 * @brief Received Topology Change Notification BPDU (8.7.2)
 * @param[in] port Pointer to the bridge port context
 * @param[in] bpdu Pointer to the received TCN BPDU
 **/

void stpReceivedTcnBpdu(StpBridgePort *port, const StpBpdu *bpdu)
{
   //Check port state
   if(port->state != STP_PORT_STATE_DISABLED && port->macOperState)
   {
      //Check whether the port on which the Topology Change Notification BPDU
      //was received is the Designated Port for the LAN to which it is attached
      if(stpDesignatedPort(port))
      {
         //The Topology Change Detection procedure is used
         stpTopologyChangeDetection(port->context);
         //The Acknowledge Topology Change procedure is used
         stpAckTopologyChange(port);
      }
   }
}


/**
 * @brief Hello Timer expiry (8.7.3)
 * @param[in] context Pointer to the STP bridge context
 **/

void stpHelloTimerExpiry(StpBridgeContext *context)
{
   //The Configuration BPDU Generation procedure is used and the Hello Timer
   //is started
   stpConfigBpduGeneration(context);
   stpStartTimer(&context->helloTimer, 0);
}


/**
 * @brief Message Age Timer expiry (8.7.4)
 * @param[in] port Pointer to the bridge port context
 **/

void stpMessageAgeTimerExpiry(StpBridgePort *port)
{
   bool_t rootBridge;
   StpBridgeContext *context;

   //Point to the STP bridge context
   context = port->context;

   //Test whether the bridge is currently the Root
   rootBridge = stpRootBridge(context);

   //The procedure to Become Designated Port is used for the port for which
   //Message Age Timer has expired
   stpBecomeDesignatedPort(port);

   //The Configuration Update procedure is used
   stpConfigUpdate(context);
   //The Port State Selection procedure is used
   stpPortStateSelection(context);

   //Check if the bridge is selected as the Root following Configuration Update
   if(!rootBridge && stpRootBridge(context))
   {
      //The Max Age, Hello Time, and Forward Delay parameters held by the
      //bridge are set to the values of the Bridge Max Age, Bridge Hello Time,
      //and Bridge Forward Delay parameters
      context->maxAge = context->bridgeMaxAge;
      context->helloTime = context->bridgeHelloTime;
      context->forwardDelay = context->bridgeForwardDelay;

      //The Topology Change Detection procedure is used
      stpTopologyChangeDetection(context);
      //The Topology Change Notification Timer is stopped
      stpStopTimer(&context->tcnTimer);

      //The Configuration BPDU Generation procedure is used and the Hello
      //Timer is started
      stpConfigBpduGeneration(context);
      stpStartTimer(&context->helloTimer, 0);
   }
}


/**
 * @brief Forward Delay Timer expiry (8.7.5)
 * @param[in] port Pointer to the bridge port context
 **/

void stpForwardDelayTimerExpiry(StpBridgePort *port)
{
   StpBridgeContext *context;

   //Point to the STP bridge context
   context = port->context;

   //Check the state of the port
   if(port->state == STP_PORT_STATE_LISTENING)
   {
      //If the state of the port for which the Forward Delay Timer has expired
      //was Listening, then the Port State is set to Learning
      stpUpdatePortState(port, STP_PORT_STATE_LEARNING);

      //The Forward Delay Timer is restarted
      stpStartTimer(&port->forwardDelayTimer, 0);
   }
   else if(port->state == STP_PORT_STATE_LEARNING)
   {
      //If the state of the port for which the Forward Delay Timer has expired
      //was Learning, then the Port State is set to Forwarding
      stpUpdatePortState(port, STP_PORT_STATE_FORWARDING);

      //Check if the bridge is the Designated Bridge for at least one of the
      //LANs to which its ports are attached and the Change Detection Enabled
      //parameter for the port is set
      if(stpDesignatedBridge(context) && port->changeDetectionEnabled)
      {
         //The Topology Change Detection procedure is invoked
         stpTopologyChangeDetection(context);
      }
   }
   else
   {
      //Just for sanity
   }
}


/**
 * @brief Topology Change Notification Timer expiry (8.7.6)
 * @param[in] context Pointer to the STP bridge context
 **/

void stpTcnTimerExpiry(StpBridgeContext *context)
{
   //The Transmit Topology Change Notification BPDU procedure is used
   stpTransmitTcnBpdu(context);
   //The Topology Change Notification Timer is restarted
   stpStartTimer(&context->tcnTimer, 0);
}


/**
 * @brief Topology Change Timer expiry (8.7.7)
 * @param[in] context Pointer to the STP bridge context
 **/

void stpTopologyChangeTimerExpiry(StpBridgeContext *context)
{
   //The Topology Change Detected flag parameter held by the Bridge is reset
   context->topologyChangeDetected = FALSE;
   //The Topology Change flag parameter held by the Bridge is reset
   stpUpdateTopologyChange(context, FALSE);
}


/**
 * @brief Hold Timer expiry (8.7.8)
 * @param[in] port Pointer to the bridge port context
 **/

void stpHoldTimerExpiry(StpBridgePort *port)
{
   //If the Configuration Pending flag parameter for the port for which the
   //Hold Timer has expired is set, the Transmit Configuration BPDU procedure
   //is invoked for that port
   if(port->configPending)
   {
      stpTransmitConfigBpdu(port);
   }
}

#endif
