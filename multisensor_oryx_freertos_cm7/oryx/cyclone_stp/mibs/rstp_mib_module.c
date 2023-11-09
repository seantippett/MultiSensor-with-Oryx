/**
 * @file rstp_mib_module.c
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

//Switch to the appropriate trace level
#define TRACE_LEVEL SNMP_TRACE_LEVEL

//Dependencies
#include "core/net.h"
#include "mibs/mib_common.h"
#include "mibs/rstp_mib_module.h"
#include "mibs/rstp_mib_impl.h"
#include "core/crypto.h"
#include "encoding/asn1.h"
#include "encoding/oid.h"
#include "debug.h"

//Check TCP/IP stack configuration
#if (RSTP_MIB_SUPPORT == ENABLED)


/**
 * @brief RSTP MIB base
 **/

RstpMibBase rstpMibBase;


/**
 * @brief RSTP MIB objects
 **/

const MibObject rstpMibObjects[] =
{
   //dot1dStpVersion object (1.3.6.1.2.1.17.2.16)
   {
      "dot1dStpVersion",
      {43, 6, 1, 2, 1, 17, 2, 16},
      8,
      ASN1_CLASS_UNIVERSAL,
      ASN1_TYPE_INTEGER,
      MIB_ACCESS_READ_WRITE,
      NULL,
      NULL,
      sizeof(int32_t),
      rstpMibSetDot1dStpVersion,
      rstpMibGetDot1dStpVersion,
      NULL
   },
   //dot1dStpTxHoldCount object (1.3.6.1.2.1.17.2.17)
   {
      "dot1dStpTxHoldCount",
      {43, 6, 1, 2, 1, 17, 2, 17},
      8,
      ASN1_CLASS_UNIVERSAL,
      ASN1_TYPE_INTEGER,
      MIB_ACCESS_READ_WRITE,
      NULL,
      NULL,
      sizeof(int32_t),
      rstpMibSetDot1dStpTxHoldCount,
      rstpMibGetDot1dStpTxHoldCount,
      NULL
   },
   //dot1dStpPortProtocolMigration object (1.3.6.1.2.1.17.2.19.1.1)
   {
      "dot1dStpPortProtocolMigration",
      {43, 6, 1, 2, 1, 17, 2, 19, 1, 1},
      10,
      ASN1_CLASS_UNIVERSAL,
      ASN1_TYPE_INTEGER,
      MIB_ACCESS_READ_WRITE,
      NULL,
      NULL,
      sizeof(int32_t),
      rstpMibSetDot1dStpExtPortEntry,
      rstpMibGetDot1dStpExtPortEntry,
      rstpMibGetNextDot1dStpExtPortEntry
   },
   //dot1dStpPortAdminEdgePort object (1.3.6.1.2.1.17.2.19.1.2)
   {
      "dot1dStpPortAdminEdgePort",
      {43, 6, 1, 2, 1, 17, 2, 19, 1, 2},
      10,
      ASN1_CLASS_UNIVERSAL,
      ASN1_TYPE_INTEGER,
      MIB_ACCESS_READ_WRITE,
      NULL,
      NULL,
      sizeof(int32_t),
      rstpMibSetDot1dStpExtPortEntry,
      rstpMibGetDot1dStpExtPortEntry,
      rstpMibGetNextDot1dStpExtPortEntry
   },
   //dot1dStpPortOperEdgePort object (1.3.6.1.2.1.17.2.19.1.3)
   {
      "dot1dStpPortOperEdgePort",
      {43, 6, 1, 2, 1, 17, 2, 19, 1, 3},
      10,
      ASN1_CLASS_UNIVERSAL,
      ASN1_TYPE_INTEGER,
      MIB_ACCESS_READ_ONLY,
      NULL,
      NULL,
      sizeof(int32_t),
      NULL,
      rstpMibGetDot1dStpExtPortEntry,
      rstpMibGetNextDot1dStpExtPortEntry
   },
   //dot1dStpPortAdminPointToPoint object (1.3.6.1.2.1.17.2.19.1.4)
   {
      "dot1dStpPortAdminPointToPoint",
      {43, 6, 1, 2, 1, 17, 2, 19, 1, 4},
      10,
      ASN1_CLASS_UNIVERSAL,
      ASN1_TYPE_INTEGER,
      MIB_ACCESS_READ_WRITE,
      NULL,
      NULL,
      sizeof(int32_t),
      rstpMibSetDot1dStpExtPortEntry,
      rstpMibGetDot1dStpExtPortEntry,
      rstpMibGetNextDot1dStpExtPortEntry
   },
   //dot1dStpPortOperPointToPoint object (1.3.6.1.2.1.17.2.19.1.5)
   {
      "dot1dStpPortOperPointToPoint",
      {43, 6, 1, 2, 1, 17, 2, 19, 1, 5},
      10,
      ASN1_CLASS_UNIVERSAL,
      ASN1_TYPE_INTEGER,
      MIB_ACCESS_READ_ONLY,
      NULL,
      NULL,
      sizeof(int32_t),
      NULL,
      rstpMibGetDot1dStpExtPortEntry,
      rstpMibGetNextDot1dStpExtPortEntry
   },
   //dot1dStpPortAdminPathCost object (1.3.6.1.2.1.17.2.19.1.6)
   {
      "dot1dStpPortAdminPathCost",
      {43, 6, 1, 2, 1, 17, 2, 19, 1, 6},
      10,
      ASN1_CLASS_UNIVERSAL,
      ASN1_TYPE_INTEGER,
      MIB_ACCESS_READ_WRITE,
      NULL,
      NULL,
      sizeof(int32_t),
      rstpMibSetDot1dStpExtPortEntry,
      rstpMibGetDot1dStpExtPortEntry,
      rstpMibGetNextDot1dStpExtPortEntry
   }
};


/**
 * @brief RSTP MIB module
 **/

const MibModule rstpMibModule =
{
   "RSTP-MIB",
   {43, 6, 1, 2, 1, 129, 6},
   7,
   rstpMibObjects,
   arraysize(rstpMibObjects),
   rstpMibInit,
   NULL,
   NULL,
   NULL,
   NULL
};

#endif
