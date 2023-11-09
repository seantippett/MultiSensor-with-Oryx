/**
 * @file tls_client_extensions.h
 * @brief Formatting and parsing of extensions (TLS client)
 *
 * @section License
 *
 * Copyright (C) 2010-2023 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneSSL Eval.
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

#ifndef _TLS_CLIENT_EXTENSIONS_H
#define _TLS_CLIENT_EXTENSIONS_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS client specific functions
error_t tlsFormatClientSupportedVersionsExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatClientSniExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatClientMaxFragLenExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatClientRecordSizeLimitExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatSupportedGroupsExtension(TlsContext *context,
   uint_t cipherSuiteTypes, uint8_t *p, size_t *written);

error_t tlsFormatClientEcPointFormatsExtension(TlsContext *context,
   uint_t cipherSuiteTypes, uint8_t *p, size_t *written);

error_t tlsFormatClientAlpnExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatClientCertTypeListExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerCertTypeListExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatClientEmsExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatClientSessionTicketExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatClientRenegoInfoExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatClientHelloPaddingExtension(TlsContext *context,
   size_t clientHelloLen, uint8_t *p, size_t *written);

error_t tlsParseServerSniExtension(TlsContext *context,
   const TlsServerNameList *serverNameList);

error_t tlsParseServerMaxFragLenExtension(TlsContext *context,
   const TlsExtension *maxFragLen);

error_t tlsParseServerRecordSizeLimitExtension(TlsContext *context,
   const TlsExtension *recordSizeLimit);

error_t tlsParseServerEcPointFormatsExtension(TlsContext *context,
   const TlsEcPointFormatList *ecPointFormatList);

error_t tlsParseServerAlpnExtension(TlsContext *context,
   const TlsProtocolNameList *protocolNameList);

error_t tlsParseClientCertTypeExtension(TlsContext *context,
   const TlsExtension *clientCertType);

error_t tlsParseServerCertTypeExtension(TlsContext *context,
   const TlsExtension *serverCertType);

error_t tlsParseServerEmsExtension(TlsContext *context,
   const TlsExtension *extendedMasterSecret);

error_t tlsParseServerSessionTicketExtension(TlsContext *context,
   const TlsExtension *sessionTicket);

error_t tlsParseServerRenegoInfoExtension(TlsContext *context,
   const TlsHelloExtensions *extensions);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
