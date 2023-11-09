/**
 * @file snmp_agent_misc.h
 * @brief Helper functions for SNMP agent
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

#ifndef _SNMP_AGENT_MISC_H
#define _SNMP_AGENT_MISC_H

//Dependencies
#include "core/net.h"
#include "snmp/snmp_agent.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//SNMP agent related functions
void snmpLockMib(SnmpAgentContext *context);
void snmpUnlockMib(SnmpAgentContext *context);

SnmpUserEntry *snmpCreateCommunityEntry(SnmpAgentContext *context);

SnmpUserEntry *snmpFindCommunityEntry(SnmpAgentContext *context,
   const char_t *community, size_t length);

error_t snmpParseVarBinding(const uint8_t *p,
   size_t length, SnmpVarBind *var, size_t *consumed);

error_t snmpWriteVarBinding(SnmpAgentContext *context, const SnmpVarBind *var);
error_t snmpCopyVarBindingList(SnmpAgentContext *context);

error_t snmpWriteTrapVarBindingList(SnmpAgentContext *context,
   uint_t genericTrapType, uint_t specificTrapCode,
   const SnmpTrapObject *objectList, uint_t objectListSize);

error_t snmpTranslateStatusCode(SnmpMessage *message, error_t status, uint_t index);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
