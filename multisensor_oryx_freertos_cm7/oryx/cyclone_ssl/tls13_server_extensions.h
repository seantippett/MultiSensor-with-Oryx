/**
 * @file tls13_server_extensions.h
 * @brief Formatting and parsing of extensions (TLS 1.3 server)
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

#ifndef _TLS13_SERVER_EXTENSIONS_H
#define _TLS13_SERVER_EXTENSIONS_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS 1.3 server specific functions
error_t tls13FormatServerSupportedVersionsExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tls13FormatSelectedGroupExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tls13FormatServerKeyShareExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tls13FormatServerPreSharedKeyExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tls13FormatServerEarlyDataExtension(TlsContext *context,
   TlsMessageType msgType, uint8_t *p, size_t *written);

error_t tls13ParseClientKeyShareExtension(TlsContext *context,
   const Tls13KeyShareList *keyShareList);

error_t tls13ParsePskKeModesExtension(TlsContext *context,
   const Tls13PskKeModeList *pskKeModeList);

error_t tls13ParseClientPreSharedKeyExtension(TlsContext *context,
   const TlsClientHello *clientHello, size_t clientHelloLen,
   const Tls13PskIdentityList *identityList, const Tls13PskBinderList *binderList);

error_t tls13ParseClientEarlyDataExtension(TlsContext *context,
   const TlsExtension *earlyDataIndication);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
