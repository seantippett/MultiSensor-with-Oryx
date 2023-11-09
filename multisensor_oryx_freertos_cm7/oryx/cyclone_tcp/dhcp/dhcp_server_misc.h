/**
 * @file dhcp_server_misc.h
 * @brief Helper functions for DHCP server
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

#ifndef _DHCP_SERVER_MISC_H
#define _DHCP_SERVER_MISC_H

//Dependencies
#include "core/net.h"
#include "dhcp/dhcp_server.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//Tick counter to handle periodic operations
extern systime_t dhcpServerTickCounter;

//DHCP server related functions
void dhcpServerTick(DhcpServerContext *context);

void dhcpServerProcessMessage(NetInterface *interface,
   const IpPseudoHeader *pseudoHeader, const UdpHeader *udpHeader,
   const NetBuffer *buffer, size_t offset, const NetRxAncillary *ancillary,
   void *param);

void dhcpServerParseDiscover(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

void dhcpServerParseRequest(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

void dhcpServerParseDecline(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

void dhcpServerParseRelease(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

void dhcpServerParseInform(DhcpServerContext *context,
   const DhcpMessage *message, size_t length);

error_t dhcpServerSendReply(DhcpServerContext *context, uint8_t type,
   Ipv4Addr yourIpAddr, const DhcpMessage *request, size_t requestLen);

DhcpServerBinding *dhcpServerCreateBinding(DhcpServerContext *context);

DhcpServerBinding *dhcpServerFindBindingByMacAddr(DhcpServerContext *context,
   const MacAddr *macAddr);

DhcpServerBinding *dhcpServerFindBindingByIpAddr(DhcpServerContext *context,
   Ipv4Addr ipAddr);

error_t dhcpServerGetNextIpAddr(DhcpServerContext *context, Ipv4Addr *ipAddr);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
