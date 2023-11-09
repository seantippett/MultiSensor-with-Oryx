/**
 * @file dtls_record.h
 * @brief DTLS record protocol
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

#ifndef _DTLS_RECORD_H
#define _DTLS_RECORD_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//DTLS related functions
error_t dtlsWriteProtocolData(TlsContext *context,
   const uint8_t *data, size_t length, TlsContentType contentType);

error_t dtlsReadProtocolData(TlsContext *context,
   uint8_t **data, size_t *length, TlsContentType *contentType);

error_t dtlsWriteRecord(TlsContext *context, const uint8_t *data,
   size_t length, TlsContentType contentType);

error_t dtlsReadRecord(TlsContext *context);
error_t dtlsProcessRecord(TlsContext *context);

error_t dtlsSendFlight(TlsContext *context);

error_t dtlsFragmentHandshakeMessage(TlsContext *context, uint16_t version,
   TlsEncryptionEngine *encryptionEngine, const DtlsHandshake *message);

error_t dtlsReassembleHandshakeMessage(TlsContext *context,
   const DtlsHandshake *message);

error_t dtlsReadDatagram(TlsContext *context, uint8_t *data,
   size_t size, size_t *length);

error_t dtlsTick(TlsContext *context);

void dtlsIncSequenceNumber(DtlsSequenceNumber *seqNum);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
