/**
 * @file tls13_client_extensions.h
 * @brief Formatting and parsing of extensions (TLS 1.3 client)
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

#ifndef _TLS13_CLIENT_EXTENSIONS_H
#define _TLS13_CLIENT_EXTENSIONS_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS 1.3 client specific functions
error_t tls13FormatCookieExtension(TlsContext *context, uint8_t *p,
   size_t *written);

error_t tls13FormatClientKeyShareExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tls13FormatPskKeModesExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tls13FormatClientPreSharedKeyExtension(TlsContext *context,
   uint8_t *p, size_t *written, Tls13PskIdentityList **identityList,
   Tls13PskBinderList **binderList);

error_t tls13FormatClientEarlyDataExtension(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tls13ParseServerSupportedVersionsExtension(TlsContext *context,
   const TlsExtension *selectedVersion);

error_t tls13ParseCookieExtension(TlsContext *context,
   const Tls13Cookie *cookie);

error_t tls13ParseSelectedGroupExtension(TlsContext *context,
   const TlsExtension *selectedGroup);

error_t tls13ParseServerKeyShareExtension(TlsContext *context,
   const Tls13KeyShareEntry *serverShare);

error_t tls13ParseServerPreSharedKeyExtension(TlsContext *context,
   const TlsExtension *selectedIdentity);

error_t tls13ParseServerEarlyDataExtension(TlsContext *context,
   TlsMessageType msgType, const TlsExtension *earlyDataIndication);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
