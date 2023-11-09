/**
 * @file llmnr_common.c
 * @brief Definitions common to LLMNR client and responder
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

//Switch to the appropriate trace level
#define TRACE_LEVEL LLMNR_TRACE_LEVEL

//Dependencies
#include "core/net.h"
#include "llmnr/llmnr_client.h"
#include "llmnr/llmnr_responder.h"
#include "llmnr/llmnr_common.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (LLMNR_CLIENT_SUPPORT == ENABLED || LLMNR_RESPONDER_SUPPORT == ENABLED)

//LLMNR IPv6 multicast group (ff02::1:3)
const Ipv6Addr LLMNR_IPV6_MULTICAST_ADDR =
   IPV6_ADDR(0xFF02, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0003);

#endif
