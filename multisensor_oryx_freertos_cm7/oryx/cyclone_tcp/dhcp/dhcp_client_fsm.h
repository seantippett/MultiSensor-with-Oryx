/**
 * @file dhcp_client_fsm.h
 * @brief DHCP client finite state machine
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

#ifndef _DHCP_CLIENT_FSM_H
#define _DHCP_CLIENT_FSM_H

//Dependencies
#include "core/net.h"
#include "dhcp/dhcp_client.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//DHCP client related functions
void dhcpClientStateInit(DhcpClientContext *context);
void dhcpClientStateSelecting(DhcpClientContext *context);
void dhcpClientStateRequesting(DhcpClientContext *context);
void dhcpClientStateInitReboot(DhcpClientContext *context);
void dhcpClientStateRebooting(DhcpClientContext *context);
void dhcpClientStateProbing(DhcpClientContext *context);
void dhcpClientStateAnnouncing(DhcpClientContext *context);
void dhcpClientStateBound(DhcpClientContext *context);
void dhcpClientStateRenewing(DhcpClientContext *context);
void dhcpClientStateRebinding(DhcpClientContext *context);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
