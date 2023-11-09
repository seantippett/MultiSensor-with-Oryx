/**
 * @file tls_misc.h
 * @brief TLS helper functions
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

#ifndef _TLS_MISC_H
#define _TLS_MISC_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS related functions
void tlsChangeState(TlsContext *context, TlsState newState);
void tlsProcessError(TlsContext *context, error_t errorCode);

error_t tlsGenerateRandomValue(TlsContext *context, uint8_t *random);
error_t tlsGenerateSessionId(TlsContext *context, size_t length);

error_t tlsSelectVersion(TlsContext *context, uint16_t version);
error_t tlsSelectCipherSuite(TlsContext *context, uint16_t identifier);

error_t tlsSaveSessionId(const TlsContext *context,
   TlsSessionState *session);

error_t tlsSaveSessionTicket(const TlsContext *context,
   TlsSessionState *session);

error_t tlsRestoreSessionId(TlsContext *context,
   const TlsSessionState *session);

error_t tlsRestoreSessionTicket(TlsContext *context,
   const TlsSessionState *session);

error_t tlsInitEncryptionEngine(TlsContext *context,
   TlsEncryptionEngine *encryptionEngine, TlsConnectionEnd entity,
   const uint8_t *secret);

void tlsFreeEncryptionEngine(TlsEncryptionEngine *encryptionEngine);

error_t tlsWriteMpi(const Mpi *a, uint8_t *data, size_t *length);
error_t tlsReadMpi(Mpi *a, const uint8_t *data, size_t size, size_t *length);

error_t tlsWriteEcPoint(const EcDomainParameters *params,
   const EcPoint *a, uint8_t *data, size_t *length);

error_t tlsReadEcPoint(const EcDomainParameters *params,
   EcPoint *a, const uint8_t *data, size_t size, size_t *length);

const char_t *tlsGetVersionName(uint16_t version);
const HashAlgo *tlsGetHashAlgo(uint8_t hashAlgoId);
const EcCurveInfo *tlsGetCurveInfo(TlsContext *context, uint16_t namedCurve);
TlsNamedGroup tlsGetNamedCurve(const uint8_t *oid, size_t length);

size_t tlsComputeEncryptionOverhead(TlsEncryptionEngine *encryptionEngine,
   size_t payloadLen);

bool_t tlsCheckDnsHostname(const char_t *name, size_t length);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
