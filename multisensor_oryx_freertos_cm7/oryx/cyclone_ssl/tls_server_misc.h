/**
 * @file tls_server_misc.h
 * @brief Helper functions for TLS server
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

#ifndef _TLS_SERVER_MISC_H
#define _TLS_SERVER_MISC_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS server specific functions
error_t tlsFormatPskIdentityHint(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsFormatServerKeyParams(TlsContext *context,
   uint8_t *p, size_t *written);

error_t tlsGenerateServerKeySignature(TlsContext *context,
   TlsDigitalSignature *signature, const uint8_t *params,
   size_t paramsLen, size_t *written);

error_t tls12GenerateServerKeySignature(TlsContext *context,
   Tls12DigitalSignature *signature, const uint8_t *params,
   size_t paramsLen, size_t *written);

error_t tlsCheckSignalingCipherSuiteValues(TlsContext *context,
   const TlsCipherSuites *cipherSuites);

error_t tlsResumeStatefulSession(TlsContext *context, const uint8_t *sessionId,
   size_t sessionIdLen, const TlsCipherSuites *cipherSuites,
   const TlsHelloExtensions *extensions);

error_t tlsResumeStatelessSession(TlsContext *context, const uint8_t *sessionId,
   size_t sessionIdLen, const TlsCipherSuites *cipherSuites,
   const TlsHelloExtensions *extensions);

error_t tlsNegotiateVersion(TlsContext *context, uint16_t clientVersion,
   const TlsSupportedVersionList *supportedVersionList);

error_t tlsNegotiateCipherSuite(TlsContext *context, const HashAlgo *hashAlgo,
   const TlsCipherSuites *cipherSuites, TlsHelloExtensions *extensions);

error_t tlsSelectGroup(TlsContext *context,
   const TlsSupportedGroupList *groupList);

error_t tlsSelectEcdheGroup(TlsContext *context,
   const TlsSupportedGroupList *groupList);

error_t tlsSelectCertificate(TlsContext *context,
   const TlsHelloExtensions *extensions);

error_t tlsParseCompressMethods(TlsContext *context,
   const TlsCompressMethods *compressMethods);

error_t tlsParsePskIdentity(TlsContext *context,
   const uint8_t *p, size_t length, size_t *consumed);

error_t tlsParseClientKeyParams(TlsContext *context,
   const uint8_t *p, size_t length, size_t *consumed);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
