/**
 * @file tls_record.h
 * @brief TLS record protocol
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

#ifndef _TLS_RECORD_H
#define _TLS_RECORD_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS related functions
error_t tlsWriteProtocolData(TlsContext *context,
   const uint8_t *data, size_t length, TlsContentType contentType);

error_t tlsReadProtocolData(TlsContext *context,
   uint8_t **data, size_t *length, TlsContentType *contentType);

error_t tlsWriteRecord(TlsContext *context, const uint8_t *data,
   size_t length, TlsContentType contentType);

error_t tlsReadRecord(TlsContext *context, uint8_t *data,
   size_t size, size_t *length, TlsContentType *contentType);

error_t tlsProcessRecord(TlsContext *context, TlsRecord *record);

void tlsSetRecordType(TlsContext *context, void *record, uint8_t type);
uint8_t tlsGetRecordType(TlsContext *context, void *record);
void tlsSetRecordLength(TlsContext *context, void *record, size_t length);
size_t tlsGetRecordLength(TlsContext *context, void *record);
uint8_t *tlsGetRecordData(TlsContext *context, void *record);

void tlsFormatAad(TlsContext *context, TlsEncryptionEngine *encryptionEngine,
   const void *record, uint8_t *aad, size_t *aadLen);

void tlsFormatNonce(TlsContext *context, TlsEncryptionEngine *encryptionEngine,
   const void *record, const uint8_t *recordIv, uint8_t *nonce, size_t *nonceLen);

void tlsIncSequenceNumber(TlsSequenceNumber *seqNum);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
