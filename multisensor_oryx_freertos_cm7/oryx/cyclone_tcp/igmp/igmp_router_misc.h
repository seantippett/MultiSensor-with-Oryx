/**
 * @file igmp_router_misc.h
 * @brief Helper functions fore IGMP router
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

#ifndef _IGMP_ROUTER_MISC_H
#define _IGMP_ROUTER_MISC_H

//Dependencies
#include "core/net.h"
#include "igmp/igmp_router.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//IGMP router related functions
error_t igmpRouterSendGeneralQuery(IgmpRouterContext *context);

error_t igmpRouterSendGroupSpecificQuery(IgmpRouterContext *context,
   Ipv4Addr groupAddr);

error_t igmpRouterSendMembershipQuery(IgmpRouterContext *context,
   Ipv4Addr destAddr, Ipv4Addr groupAddr, systime_t maxRespTime);

void igmpRouterProcessMessage(IgmpRouterContext *context,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length);

void igmpRouterProcessMembershipQuery(IgmpRouterContext *context,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length);

void igmpRouterProcessMembershipReport(IgmpRouterContext *context,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length);

void igmpRouterProcessLeaveGroup(IgmpRouterContext *context,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length);

IgmpRouterGroup *igmpRouterCreateGroup(IgmpRouterContext *context,
   Ipv4Addr groupAddr);

IgmpRouterGroup *igmpRouterFindGroup(IgmpRouterContext *context,
   Ipv4Addr groupAddr);

void igmpRouterDeleteGroup(IgmpRouterContext *context, IgmpRouterGroup *group);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
