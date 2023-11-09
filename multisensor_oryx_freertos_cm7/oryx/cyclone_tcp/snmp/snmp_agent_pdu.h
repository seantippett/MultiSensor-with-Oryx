/**
 * @file snmp_agent_pdu.h
 * @brief SNMP agent (PDU processing)
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

#ifndef _SNMP_AGENT_PDU_H
#define _SNMP_AGENT_PDU_H

//Dependencies
#include "core/net.h"
#include "snmp/snmp_agent.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//SNMP agent related functions
error_t snmpProcessPdu(SnmpAgentContext *context);

error_t snmpProcessGetRequestPdu(SnmpAgentContext *context);
error_t snmpProcessGetBulkRequestPdu(SnmpAgentContext *context);
error_t snmpProcessSetRequestPdu(SnmpAgentContext *context);

error_t snmpFormatReportPdu(SnmpAgentContext *context, error_t errorIndication);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
