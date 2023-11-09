/**
 * @file igmp_host.h
 * @brief IGMP host
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

#ifndef _IGMP_HOST_H
#define _IGMP_HOST_H

//Dependencies
#include "core/net.h"
#include "igmp/igmp_common.h"

//IGMP host support
#ifndef IGMP_HOST_SUPPORT
   #define IGMP_HOST_SUPPORT ENABLED
#elif (IGMP_HOST_SUPPORT != ENABLED && IGMP_HOST_SUPPORT != DISABLED)
   #error IGMP_HOST_SUPPORT parameter is not valid
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Multicast group states
 **/

typedef enum
{
   IGMP_HOST_GROUP_STATE_NON_MEMBER      = 0,
   IGMP_HOST_GROUP_STATE_DELAYING_MEMBER = 1,
   IGMP_HOST_GROUP_STATE_IDLE_MEMBER     = 2
} IgmpHostGroupState;


/**
 * @brief IGMP host context
 **/

typedef struct
{
   bool_t igmpv1RouterPresent; ///<An IGMPv1 query has been recently heard
   NetTimer timer;             ///<IGMPv1 router present timer
} IgmpHostContext;


//IGMP host related functions
error_t igmpHostInit(NetInterface *interface);
error_t igmpHostJoinGroup(NetInterface *interface, Ipv4FilterEntry *entry);
error_t igmpHostLeaveGroup(NetInterface *interface, Ipv4FilterEntry *entry);

void igmpHostTick(NetInterface *interface);
void igmpHostLinkChangeEvent(NetInterface *interface);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
