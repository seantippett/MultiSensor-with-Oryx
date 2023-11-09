/**
 * @file tls_transcript_hash.h
 * @brief Transcript hash calculation
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

#ifndef _TLS_TRANSCRIPT_HASH_H
#define _TLS_TRANSCRIPT_HASH_H

//Dependencies
#include "tls.h"

//C++ guard
#ifdef __cplusplus
extern "C" {
#endif

//TLS related functions
error_t tlsInitTranscriptHash(TlsContext *context);

void tlsUpdateTranscriptHash(TlsContext *context, const void *data,
   size_t length);

error_t tlsFinalizeTranscriptHash(TlsContext *context, const HashAlgo *hash,
   const void *hashContext, const char_t *label, uint8_t *output);

void tlsFreeTranscriptHash(TlsContext *context);

error_t tlsComputeVerifyData(TlsContext *context, TlsConnectionEnd entity,
   uint8_t *verifyData, size_t *verifyDataLen);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif
