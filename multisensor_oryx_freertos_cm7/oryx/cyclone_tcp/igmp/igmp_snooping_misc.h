/**
 * @file igmp_snooping_misc.h
 * @brief Helper functions for IGMP snooping switch
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

#ifndef _IGMP_SNOOPING_MISC_H
#define _IGMP_SNOOPING_MISC_H

//Dependencies
#include "core/net.h"
#include "igmp/igmp_snooping.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//IGMP snooping related functions
void igmpSnoopingProcessMessage(IgmpSnoopingContext *context,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length, const NetRxAncillary *ancillary);

void igmpSnoopingProcessMembershipQuery(IgmpSnoopingContext *context,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length, const NetRxAncillary *ancillary);

void igmpSnoopingProcessMembershipReport(IgmpSnoopingContext *context,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length, const NetRxAncillary *ancillary);

void igmpSnoopingProcessLeaveGroup(IgmpSnoopingContext *context,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length, const NetRxAncillary *ancillary);

void igmpSnoopingProcessUnknownMessage(IgmpSnoopingContext *context,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length, const NetRxAncillary *ancillary);

error_t igmpSnoopingForwardMessage(IgmpSnoopingContext *context,
   uint32_t forwardPorts, const MacAddr *destMacAddr,
   const Ipv4PseudoHeader *pseudoHeader, const IgmpMessage *message,
   size_t length);

IgmpSnoopingGroup *igmpSnoopingCreateGroup(IgmpSnoopingContext *context,
   Ipv4Addr groupAddr, uint8_t port);

IgmpSnoopingGroup *igmpSnoopingFindGroup(IgmpSnoopingContext *context,
   Ipv4Addr groupAddr, uint8_t port);

void igmpSnoopingDeleteGroup(IgmpSnoopingContext *context,
   IgmpSnoopingGroup *group);

void igmpSnoopingEnableMonitoring(IgmpSnoopingContext *context, bool_t enable);

void igmpSnoopingUpdateStaticFdbEntry(IgmpSnoopingContext *context,
   Ipv4Addr groupAddr);

void igmpSnoopingSetUnknownMcastFwdPorts(IgmpSnoopingContext *context,
   bool_t enable, uint32_t forwardPorts);

uint32_t igmpSnoopingGetRouterPorts(IgmpSnoopingContext *context);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
