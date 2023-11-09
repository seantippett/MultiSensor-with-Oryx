/**
 * @file tls_client_misc.h
 * @brief Helper functions for TLS client
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

#ifndef _TLS_CLIENT_MISC_H
#define _TLS_CLIENT_MISC_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS client specific functions
error_t tlsFormatInitialClientHello(TlsContext *context);

error_t tlsFormatSessionId(TlsContext *context, uint8_t *p,
   size_t *written);

error_t tlsFormatCipherSuites(TlsContext *context, uint_t *cipherSuiteTypes,
   uint8_t *p, size_t *written);

error_t tlsFormatCompressMethods(TlsContext *context, uint8_t *p,
   size_t *written);

error_t tlsFormatPskIdentity(TlsContext *context, uint8_t *p,
   size_t *written);

error_t tlsFormatClientKeyParams(TlsContext *context, uint8_t *p,
   size_t *written);

error_t tlsParsePskIdentityHint(TlsContext *context, const uint8_t *p,
   size_t length, size_t *consumed);

error_t tlsParseServerKeyParams(TlsContext *context, const uint8_t *p,
   size_t length, size_t *consumed);

error_t tlsVerifyServerKeySignature(TlsContext *context,
   const TlsDigitalSignature *signature, size_t length,
   const uint8_t *params, size_t paramsLen, size_t *consumed);

error_t tls12VerifyServerKeySignature(TlsContext *context,
   const Tls12DigitalSignature *signature, size_t length,
   const uint8_t *params, size_t paramsLen, size_t *consumed);

error_t tlsSelectClientVersion(TlsContext *context,
   const TlsServerHello *message, const TlsHelloExtensions *extensions);

error_t tlsResumeSession(TlsContext *context, const uint8_t *sessionId,
   size_t sessionIdLen, uint16_t cipherSuite);

bool_t tlsIsTicketValid(TlsContext *context);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
