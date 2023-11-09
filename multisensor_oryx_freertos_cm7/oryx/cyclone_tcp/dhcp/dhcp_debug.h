/**
 * @file dhcp_debug.h
 * @brief Data logging functions for debugging purpose (DHCP)
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

#ifndef _DHCP_DEBUG_H
#define _DHCP_DEBUG_H

//Dependencies
#include "core/net.h"
#include "dhcp/dhcp_common.h"
#include "debug.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//Check current trace level
#if (DHCP_TRACE_LEVEL >= TRACE_LEVEL_DEBUG)
   error_t dhcpDumpMessage(const DhcpMessage *message, size_t length);
   error_t dhcpDumpMessageType(const DhcpOption *option);
   error_t dhcpDumpParamRequestList(const DhcpOption *option);
   error_t dhcpDumpBoolean(const DhcpOption *option);
   error_t dhcpDumpInt8(const DhcpOption *option);
   error_t dhcpDumpInt16(const DhcpOption *option);
   error_t dhcpDumpInt32(const DhcpOption *option);
   error_t dhcpDumpString(const DhcpOption *option);
   error_t dhcpDumpIpv4Addr(const DhcpOption *option);
   error_t dhcpDumpIpv4AddrList(const DhcpOption *option);
   error_t dhcpDumpRawData(const DhcpOption *option);
#else
   #define dhcpDumpMessage(message, length)
#endif

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
