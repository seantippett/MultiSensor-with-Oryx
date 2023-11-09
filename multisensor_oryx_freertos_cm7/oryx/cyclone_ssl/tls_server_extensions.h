/**
 * @file tls_server_extensions.h
 * @brief Formatting and parsing of extensions (TLS server)
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

#ifndef _TLS_SERVER_EXTENSIONS_H
#define _TLS_SERVER_EXTENSIONS_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS server specific functions
error_t tlsFormatServerSniExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerMaxFragLenExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerRecordSizeLimitExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerEcPointFormatsExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerAlpnExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatClientCertTypeExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerCertTypeExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerEmsExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerSessionTicketExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerRenegoInfoExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsParseClientSupportedVersionsExtension(TlsContext *context,
   const TlsSupportedVersionList *supportedVersionList);

error_t tlsParseClientSniExtension(TlsContext *context,
   const TlsServerNameList *serverNameList);

error_t tlsParseClientMaxFragLenExtension(TlsContext *context,
   const TlsExtension *maxFragLen);

error_t tlsParseClientRecordSizeLimitExtension(TlsContext *context,
   const TlsExtension *recordSizeLimit);

error_t tlsParseClientEcPointFormatsExtension(TlsContext *context,
   const TlsEcPointFormatList *ecPointFormatList);

error_t tlsParseClientAlpnExtension(TlsContext *context,
   const TlsProtocolNameList *protocolNameList);

error_t tlsParseClientCertTypeListExtension(TlsContext *context,
   const TlsCertTypeList *clientCertTypeList);

error_t tlsParseServerCertTypeListExtension(TlsContext *context,
   const TlsCertTypeList *serverCertTypeList);

error_t tlsParseClientEmsExtension(TlsContext *context,
   const TlsExtension *extendedMasterSecret);

error_t tlsParseClientSessionTicketExtension(TlsContext *context,
   const TlsExtension *sessionTicket);

error_t tlsParseClientRenegoInfoExtension(TlsContext *context,
   const TlsHelloExtensions *extensions);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
