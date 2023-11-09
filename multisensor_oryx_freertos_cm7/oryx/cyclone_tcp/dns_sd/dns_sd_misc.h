/**
 * @file dns_sd_misc.h
 * @brief Helper functions for DNS-SD
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

#ifndef _DNS_SD_MISC_H
#define _DNS_SD_MISC_H

//Dependencies
#include "core/net.h"
#include "dns_sd/dns_sd.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//DNS-SD related functions
void dnsSdChangeState(DnsSdContext *context, MdnsState newState,
   systime_t delay);

void dnsSdChangeInstanceName(DnsSdContext *context);

error_t dnsSdSendProbe(DnsSdContext *context);
error_t dnsSdSendAnnouncement(DnsSdContext *context);
error_t dnsSdSendGoodbye(DnsSdContext *context, const DnsSdService *service);

error_t dnsSdParseQuestion(NetInterface *interface, const MdnsMessage *query,
   size_t offset, const DnsQuestion *question, MdnsMessage *response);

void dnsSdParseNsRecord(NetInterface *interface, const MdnsMessage *query,
   size_t offset, const DnsResourceRecord *record);

void dnsSdParseAnRecord(NetInterface *interface, const MdnsMessage *response,
   size_t offset, const DnsResourceRecord *record);

void dnsSdGenerateAdditionalRecords(NetInterface *interface,
   MdnsMessage *response, bool_t legacyUnicast);

error_t dnsSdFormatServiceEnumPtrRecord(NetInterface *interface,
   MdnsMessage *message, const DnsSdService *service, uint32_t ttl);

error_t dnsSdFormatPtrRecord(NetInterface *interface, MdnsMessage *message,
   const DnsSdService *service, uint32_t ttl);

error_t dnsSdFormatSrvRecord(NetInterface *interface, MdnsMessage *message,
   const DnsSdService *service, bool_t cacheFlush, uint32_t ttl);

error_t dnsSdFormatTxtRecord(NetInterface *interface, MdnsMessage *message,
   const DnsSdService *service, bool_t cacheFlush, uint32_t ttl);

error_t dnsSdFormatNsecRecord(NetInterface *interface, MdnsMessage *message,
   const DnsSdService *service, bool_t cacheFlush, uint32_t ttl);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
