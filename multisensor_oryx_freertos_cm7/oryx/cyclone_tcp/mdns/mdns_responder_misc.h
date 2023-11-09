/**
 * @file mdns_responder_misc.h
 * @brief Helper functions for mDNS responder
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

#ifndef _MDNS_RESPONDER_MISC_H
#define _MDNS_RESPONDER_MISC_H

//Dependencies
//Dependencies
#include "core/net.h"
#include "mdns/mdns_client.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//mDNS related functions
void mdnsResponderChangeState(MdnsResponderContext *context,
   MdnsState newState, systime_t delay);

void mdnsResponderChangeHostname(MdnsResponderContext *context);

error_t mdnsResponderSendProbe(MdnsResponderContext *context);
error_t mdnsResponderSendAnnouncement(MdnsResponderContext *context);
error_t mdnsResponderSendGoodbye(MdnsResponderContext *context);

void mdnsResponderProcessQuery(NetInterface *interface, MdnsMessage *query);

error_t mdnsResponderParseQuestion(NetInterface *interface,
   const MdnsMessage *query, size_t offset, const DnsQuestion *question,
   MdnsMessage *response);

void mdnsResponderParseKnownAnRecord(NetInterface *interface,
   const MdnsMessage *query, size_t queryOffset,
   const DnsResourceRecord *queryRecord, MdnsMessage *response);

void mdnsResponderParseAnRecord(NetInterface *interface,
   const MdnsMessage *response, size_t offset, const DnsResourceRecord *record);

void mdnsResponderParseNsRecords(MdnsResponderContext *context,
   const MdnsMessage *query, size_t offset);

void mdnsResponderGenerateAdditionalRecords(MdnsResponderContext *context,
   MdnsMessage *response, bool_t legacyUnicast);

error_t mdnsResponderGenerateIpv4AddrRecords(MdnsResponderContext *context,
   MdnsMessage *message, bool_t cacheFlush, uint32_t ttl);

error_t mdnsResponderGenerateIpv6AddrRecords(MdnsResponderContext *context,
   MdnsMessage *message, bool_t cacheFlush, uint32_t ttl);

error_t mdnsResponderGenerateIpv4PtrRecords(MdnsResponderContext *context,
   MdnsMessage *message, bool_t cacheFlush, uint32_t ttl);

error_t mdnsResponderGenerateIpv6PtrRecords(MdnsResponderContext *context,
   MdnsMessage *message, bool_t cacheFlush, uint32_t ttl);

error_t mdnsResponderFormatIpv4AddrRecord(MdnsResponderContext *context,
   MdnsMessage *message, const uint8_t *ipv4Addr, bool_t cacheFlush,
   uint32_t ttl);

error_t mdnsResponderFormatIpv6AddrRecord(MdnsResponderContext *context,
   MdnsMessage *message, const uint8_t *ipv6Addr, bool_t cacheFlush,
   uint32_t ttl);

error_t mdnsResponderFormatIpv4PtrRecord(MdnsResponderContext *context,
   MdnsMessage *message, const char_t *reverseName, bool_t cacheFlush,
   uint32_t ttl);

error_t mdnsResponderFormatIpv6PtrRecord(MdnsResponderContext *context,
   MdnsMessage *message, const char_t *reverseName, bool_t cacheFlush,
   uint32_t ttl);

error_t mdnsResponderFormatNsecRecord(MdnsResponderContext *context,
   MdnsMessage *message, bool_t cacheFlush, uint32_t ttl);

DnsResourceRecord *mdnsResponderGetNextHostRecord(MdnsResponderContext *context,
   DnsResourceRecord *record);

DnsResourceRecord *mdnsResponderGetNextTiebreakerRecord(MdnsResponderContext *context,
   const MdnsMessage *query, size_t offset, DnsResourceRecord *record);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
