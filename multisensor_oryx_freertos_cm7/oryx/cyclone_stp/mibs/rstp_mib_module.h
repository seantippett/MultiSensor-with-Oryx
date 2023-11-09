/**
 * @file rstp_mib_module.h
 * @brief RSTP MIB module
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneSTP Eval.
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

#ifndef _RSTP_MIB_MODULE_H
#define _RSTP_MIB_MODULE_H

//Dependencies
#include "mibs/mib_common.h"
#include "rstp/rstp.h"

//RSTP MIB module support
#ifndef RSTP_MIB_SUPPORT
   #define RSTP_MIB_SUPPORT DISABLED
#elif (RSTP_MIB_SUPPORT != ENABLED && RSTP_MIB_SUPPORT != DISABLED)
   #error RSTP_MIB_SUPPORT parameter is not valid
#endif

//Support for SET operations
#ifndef RSTP_MIB_SET_SUPPORT
   #define RSTP_MIB_SET_SUPPORT DISABLED
#elif (RSTP_MIB_SET_SUPPORT != ENABLED && RSTP_MIB_SET_SUPPORT != DISABLED)
   #error RSTP_MIB_SET_SUPPORT parameter is not valid
#endif

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Administrative point-to-point status
 **/

typedef enum
{
   RSTP_MIB_PORT_ADMIN_P2P_FORCE_TRUE  = 0,
   RSTP_MIB_PORT_ADMIN_P2P_FORCE_FALSE = 1,
   RSTP_MIB_PORT_ADMIN_P2P_AUTO        = 2,
} RstpMibPortAdminPointToPoint;


/**
 * @brief RSTP MIB base
 **/

typedef struct
{
   RstpBridgeContext *rstpBridgeContext;
} RstpMibBase;


//RSTP MIB related constants
extern RstpMibBase rstpMibBase;
extern const MibObject rstpMibObjects[];
extern const MibModule rstpMibModule;

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
