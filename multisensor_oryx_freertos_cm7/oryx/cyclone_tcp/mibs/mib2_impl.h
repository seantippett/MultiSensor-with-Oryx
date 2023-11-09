/**
 * @file mib2_impl.h
 * @brief MIB-II module implementation
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

#ifndef _MIB2_IMPL_H
#define _MIB2_IMPL_H

//Dependencies
#include "core/net.h"
#include "mibs/mib2_module.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//MIB-II related functions
error_t mib2Init(void);

void mib2InitSysGroup(Mib2SysGroup *sysGroup);
void mib2InitIfGroup(Mib2IfGroup *ifGroup);
void mib2InitIpGroup(Mib2IpGroup *ipGroup);
void mib2InitTcpGroup(Mib2TcpGroup *tcpGroup);
void mib2InitSnmpGroup(Mib2SnmpGroup *snmpGroup);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
