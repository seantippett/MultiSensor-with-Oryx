/**
 * @file dhcp_client_misc.h
 * @brief Helper functions for DHCP client
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneTCP Eval.
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

#ifndef _DHCP_CLIENT_MISC_H
#define _DHCP_CLIENT_MISC_H

//Dependencies
#include "core/net.h"
#include "dhcp/dhcp_client.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//Tick counter to handle periodic operations
extern systime_t dhcpClientTickCounter;

//DHCP client related functions
void dhcpClientTick(DhcpClientContext *context);
void dhcpClientLinkChangeEvent(DhcpClientContext *context);

error_t dhcpClientSendDiscover(DhcpClientContext *context);
error_t dhcpClientSendRequest(DhcpClientContext *context);
error_t dhcpClientSendDecline(DhcpClientContext *context);
error_t dhcpClientSendRelease(DhcpClientContext *context);

void dhcpClientProcessMessage(NetInterface *interface,
   const IpPseudoHeader *pseudoHeader, const UdpHeader *udpHeader,
   const NetBuffer *buffer, size_t offset, const NetRxAncillary *ancillary,
   void *param);

void dhcpClientParseOffer(DhcpClientContext *context,
   const DhcpMessage *message, size_t length);

void dhcpClientParseAck(DhcpClientContext *context,
   const DhcpMessage *message, size_t length);

void dhcpClientParseNak(DhcpClientContext *context,
   const DhcpMessage *message, size_t length);

void dhcpClientCheckTimeout(DhcpClientContext *context);

uint16_t dhcpClientComputeElapsedTime(DhcpClientContext *context);

void dhcpClientChangeState(DhcpClientContext *context,
   DhcpState newState, systime_t delay);

void dhcpClientResetConfig(DhcpClientContext *context);
void dhcpClientDumpConfig(DhcpClientContext *context);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
