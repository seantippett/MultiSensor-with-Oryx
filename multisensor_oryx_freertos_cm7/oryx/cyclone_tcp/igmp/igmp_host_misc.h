/**
 * @file igmp_host_misc.h
 * @brief Helper functions for IGMP host
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

#ifndef _IGMP_HOST_MISC_H
#define _IGMP_HOST_MISC_H

//Dependencies
#include "core/net.h"
#include "igmp/igmp_host.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//IGMP host related functions
error_t igmpHostSendMembershipReport(NetInterface *interface, Ipv4Addr ipAddr);
error_t igmpHostSendLeaveGroup(NetInterface *interface, Ipv4Addr ipAddr);

void igmpHostProcessMessage(NetInterface *interface,
   const IgmpMessage *message, size_t length);

void igmpHostProcessMembershipQuery(NetInterface *interface,
   const IgmpMessage *message, size_t length);

void igmpHostProcessMembershipReport(NetInterface *interface,
   const IgmpMessage *message, size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
