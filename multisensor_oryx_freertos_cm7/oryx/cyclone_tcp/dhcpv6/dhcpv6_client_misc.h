/**
 * @file dhcpv6_client_misc.h
 * @brief Helper functions for DHCPv6 client
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

#ifndef _DHCPV6_CLIENT_MISC_H
#define _DHCPV6_CLIENT_MISC_H

//Dependencies
#include "core/net.h"
#include "dhcpv6/dhcpv6_client.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//Tick counter to handle periodic operations
extern systime_t dhcpv6ClientTickCounter;

//DHCPv6 client related functions
void dhcpv6ClientTick(Dhcpv6ClientContext *context);
void dhcpv6ClientLinkChangeEvent(Dhcpv6ClientContext *context);

error_t dhcpv6ClientSendMessage(Dhcpv6ClientContext *context,
   Dhcpv6MessageType type);

void dhcpv6ClientProcessMessage(NetInterface *interface,
   const IpPseudoHeader *pseudoHeader, const UdpHeader *udpHeader,
   const NetBuffer *buffer, size_t offset, const NetRxAncillary *ancillary,
   void *param);

void dhcpv6ClientParseAdvertise(Dhcpv6ClientContext *context,
   const Dhcpv6Message *message, size_t length);

void dhcpv6ClientParseReply(Dhcpv6ClientContext *context,
   const Dhcpv6Message *message, size_t length);

error_t dhcpv6ClientParseIaNaOption(Dhcpv6ClientContext *context,
   const Dhcpv6Option *option);

error_t dhcpv6ClientParseIaAddrOption(Dhcpv6ClientContext *context,
   const Dhcpv6Option *option);

void dhcpv6ClientAddAddr(Dhcpv6ClientContext *context, const Ipv6Addr *addr,
   uint32_t validLifetime, uint32_t preferredLifetime);

void dhcpv6ClientRemoveAddr(Dhcpv6ClientContext *context, const Ipv6Addr *addr);

void dhcpv6ClientFlushAddrList(Dhcpv6ClientContext *context);

error_t dhcpv6ClientGenerateDuid(Dhcpv6ClientContext *context);
error_t dhcpv6ClientGenerateLinkLocalAddr(Dhcpv6ClientContext *context);

bool_t dhcpv6ClientCheckServerId(Dhcpv6ClientContext *context,
   Dhcpv6Option *serverIdOption);

void dhcpv6ClientCheckTimeout(Dhcpv6ClientContext *context);

uint16_t dhcpv6ClientComputeElapsedTime(Dhcpv6ClientContext *context);

void dhcpv6ClientChangeState(Dhcpv6ClientContext *context,
   Dhcpv6State newState, systime_t delay);

void dhcpv6ClientDumpConfig(Dhcpv6ClientContext *context);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
